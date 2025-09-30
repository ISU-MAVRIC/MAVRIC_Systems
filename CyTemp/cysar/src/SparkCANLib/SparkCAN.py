from can.interface import Bus
from can import Message, CanError
from threading import Thread
import time
import queue
import traceback
import logging

from SparkCANLib import SparkController


"""
Description: Library for providing objects for controlling and receving feedback from multiple Spark Max Controllers via
CAN.
Author: Jacob Peskuski, Gabriel Carlson
"""


class SparkBus:
    def __init__(
        self, channel="can0", bustype="socketcan", bitrate=1000000, suppress_errors=True
    ):
        """
        Object for sending and receiving Spark Max CAN messages.

        @param channel: Serial channel the CAN interface is on.
        @type channel: str
        @param bustype: Type of bus, set to 'None' for to let it be resolved automatically from the default
        configuration.
        @type bustype: str
        @param bitrate: Rate at which bits are sent through the CAN bus.
        @type bitrate: int
        """
        # init CAN bus (with fallback if interface not available)
        self.simulated = False

        class _DummyBus:
            """Minimal stand‑in for python-can Bus when no CAN interface is present.
            Stores sent messages in an internal queue so recv() can return them if desired.
            """

            def __init__(self):
                self._q = queue.Queue()

            def send(self, msg):  # mimic python-can Bus API
                self._q.put(msg)

            def recv(self, timeout=0):
                if timeout is None:
                    timeout = 0
                try:
                    return self._q.get(timeout=timeout)
                except queue.Empty:
                    return None

        # optionally suppress python-can logger noise if interface missing
        if suppress_errors:
            logging.getLogger("can").setLevel(logging.CRITICAL)

        # helper for ROS logging if rclpy available
        def _ros_log(level: str, msg: str):
            try:
                from rclpy.logging import get_logger

                logger = get_logger("SparkCAN")
                if level == "info":
                    logger.info(msg)
                elif level == "warn":
                    logger.warn(msg)
                elif level == "error":
                    logger.error(msg)
                else:
                    logger.info(msg)
            except Exception:
                # fallback to print
                print(msg)

        try:
            self.bus = Bus(channel=channel, bustype=bustype, bitrate=bitrate)
            _ros_log(
                "info",
                f"[SparkCAN] Using real CAN bus: channel='{channel}', bustype='{bustype}', bitrate={bitrate} (simulation=False)",
            )
        except Exception as e:
            # Fall back to dummy bus so higher-level code keeps working.
            warn_msg = f"[SparkCAN] CAN bus init failed for channel '{channel}' ({e}). Running in simulation mode (no hardware)."
            _ros_log("warn", warn_msg)
            # Optional verbose traceback (comment out if too noisy)
            # traceback.print_exc()
            self.bus = _DummyBus()
            self.simulated = True

        # dictionary to store all of the controllers
        self.controllers = {}

        # array of all the currently added CAN IDs (Used for heartbeat)
        self.can_ids = []

        # Start heartbeat thread
        self.heartbeat_enabled = True
        self.enable_id_array = [0, 0, 0, 0, 0, 0, 0, 0]
        self.heartbeat_thread = Thread(target=self._heartbeat_runnable, daemon=True)
        self.heartbeat_thread.start()

        # Start monitor thread
        self.monitor_thread = Thread(target=self.bus_monitor, daemon=True)
        self.monitor_thread.start()

    def init_controller(self, canID):
        """
        Initializes Spark Max controllers for sending and receiving messages for a specific controller.

        @param canID: ID of the controller
        @type canID: int
        @return: Controller object pointer
        @rtype: Controller
        """

        # create new controller object, add it to list of controllers
        self.controllers.update({canID: SparkController.Controller(self, canID)})

        self.can_ids.append(canID)

        # update enable_id_array
        self._update_heartbeat_array()

        return self.controllers.get(canID)

    def send_msg(self, msg):
        """
        Sends msg to controllers via CAN bus.

        @param msg: CAN message to be sent to controller.
        @type msg: Message
        """
        try:
            # Sends the passed in CAN message to the CAN Bus initialized in constructor
            self.bus.send(msg)
        except CanError as err:
            print(err)

    def bus_monitor(self):
        """
        Thread for monitoring the bus for receivable messages.
        """

        while True:
            message = self.bus.recv(0)
            if message is None:
                # avoid tight spin when no hardware present
                if self.simulated:
                    time.sleep(0.005)
                else:
                    time.sleep(0.0005)
                continue

            # get api (class and index) and id of device from the message id
            api = (message.arbitration_id & 0x0000FFC0) >> 6
            devID = message.arbitration_id & 0x0000003F

            if (
                devID in self.controllers.keys()
                and api in self.controllers[devID].statuses.keys()
                and self.controllers[devID].statuses[api] is not None
            ):
                # using device id and api, send message to decoder
                self.controllers[devID].statuses[api].decode(message.data)

    def enable_heartbeat(self):
        """
        Enables heartbeat runnable for sending heartbeat message to CAN Bus
        """
        self.heartbeat_enabled = True

    def disable_heartbeat(self):
        """
        Disables heartbeat runnable for sending heartbeat message to CAN Bus
        """
        self.heartbeat_enabled = False

    def _update_heartbeat_array(self):
        """
        Helper method to update the heartbeat CAN message being sent when another controller is added
        """
        enable_array = ["0"] * 64
        for id in self.can_ids:
            enable_array[id] = "1"
        enable_array.reverse()
        self.enable_id_array = [0, 0, 0, 0, 0, 0, 0, 0]
        # For testing, should rewrite to clean up
        self.enable_id_array[7] = int("".join(enable_array[0:8]), 2)
        self.enable_id_array[6] = int("".join(enable_array[8:16]), 2)
        self.enable_id_array[5] = int("".join(enable_array[16:24]), 2)
        self.enable_id_array[4] = int("".join(enable_array[24:32]), 2)
        self.enable_id_array[3] = int("".join(enable_array[32:40]), 2)
        self.enable_id_array[2] = int("".join(enable_array[40:48]), 2)
        self.enable_id_array[1] = int("".join(enable_array[48:56]), 2)
        self.enable_id_array[0] = int("".join(enable_array[56:64]), 2)

    # Multithreaded runnable to continuously send heartbeat without blocking main thread. Thread started in constructor
    def _heartbeat_runnable(self):
        while True:
            if self.heartbeat_enabled:
                # set when init_controller is called
                try:
                    msg = Message(arbitration_id=0x02052480, data=self.enable_id_array)
                    self.send_msg(msg)
                except Exception as e:
                    # In simulation or if message creation fails, just log once per loop
                    print(f"[SparkCAN] Heartbeat send failed: {e}")
                time.sleep(0.02)
