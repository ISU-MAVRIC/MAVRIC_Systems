import rclpy
from rclpy.executors import MultiThreadedExecutor
from drive_system.arm_control import ArmControl
from drive_system.drive_control import DriveControl
from drive_system.steer_control import SteerControl

def main():
    rclpy.init()

    arm_control = ArmControl()
    drive_control = DriveControl()
    steer_control = SteerControl()

    executor = MultiThreadedExecutor()
    executor.add_node(arm_control)
    executor.add_node(drive_control)
    executor.add_node(steer_control)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        arm_control.destroy_node()
        drive_control.destroy_node()
        steer_control.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()