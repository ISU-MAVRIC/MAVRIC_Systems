import math
import numpy as np
# Class to contain the rover state information
"""
Default values
Wheel moment
    http://hyperphysics.phy-astr.gsu.edu/hbase/mi.html
    Cylinder = 1/2MR^2
    Placeholder value as the spokes are thick so it should act similarlly. Will need to replace with proper value later
Rolling Resistance:
    https://x-engineer.org/rolling-resistance/
    strake wheels (spiked tractor wheels) in a field 0.14-0.24
    sand 0.15-0.35
    0.15-0.2 Probably close to the actual value, but would need to measure as it is hard to calculate
"""


class Rover(object):
    def __init__(self, design_spec, energy_state=None):
        # physical
        self.g = 9.82   # gravitational constant
        self.mass = design_spec["mass"]
        self.wheel_moment = design_spec["wheel_moment"]
        self.wheel_dia = design_spec["wheel_dia"]
        self.motor_st_grad = design_spec["motor_st_grad"]
        self.mech_loss = design_spec["mech_loss"]
        self.rolling_resistance = design_spec["rolling_resistance"]
        self.battery_ah = design_spec["battery_ah"]  # Capacity in amp hours
        self.battery_v = design_spec["battery_v"]  # Voltage of entire pack in Volts
        self.motor_kv = design_spec["motor_kv"]  # Motor KV rating RPM/V
        self.motor_tc = design_spec["motor_tc"]  # Torque constant Nm/A
        self.motor_peak_current = design_spec["motor_peak_current"]
        self.gear_ratio = design_spec["gear_ratio"]
        self.max_speed_rot = self.motor_kv * self.battery_v / (60 * self.gear_ratio)   # Max RPS
        self.max_speed = self.wheel_dia * self.max_speed_rot * math.pi
        self.max_torque = self.motor_tc * self.motor_peak_current * self.gear_ratio
        self.max_force = 6 * self.max_torque / (self.wheel_dia/2)
        # energy  (units in joules)
        if energy_state is None:  # Assume battery is full charged and rover is stationary by default
            self.battery = (self.battery_v * self.battery_ah) * 3600
            self.kinetic = np.array([0, 0, 0])   # X Y Z     XY=Ground Z=Height
            self.kinetic_rot = 0
            self.wheel_speed = 0   # rotations per second
            self.path = np.array([0, 0, 0])
            self.position = np.array([0, 0, 0])
            self.velocity = np.array([0, 0, 0])
            self.rel_grav = 0  # gravitational potential energy relative to starting location
        if energy_state is not None:
            self.battery = energy_state["battery"]
            self.kinetic = energy_state["kinetic"]
            self.kinetic_rot = energy_state["kinetic_rot"]
            self.rel_grav = energy_state["relative_gravity"]

    def main_physics(self, target_position, dt):
        # Compute ideal location and energy requirement
        path = target_position - self.position
        self.path = path
        unit_path = path / np.linalg.norm(path)
        # Rolling Resistance
        # Crr * m * g * cos(angle)      cos(angle) = horizontal component magnitude / magnitude
        unit_h_path = np.linalg.norm(np.array([path[0], path[1], 0])) / (np.linalg.norm(path))
        # f_roll = crr * normal force
        f_roll = self.rolling_resistance * self.mass * self.g * unit_h_path
        f_roll_vec = f_roll * unit_path
        # print(f_roll_vec)
        # Gravity
        f_g = self.mass * self.g
        f_g_vec = np.array([0, 0, f_g])
        # Projection of gravity onto path
        f_g_path = (np.dot(f_g_vec, path) / np.dot(path, path)) * path
        # print(f_g_path)
        # Inertia
        dv = path/dt - self.velocity
        f_acc = self.mass * dv
        # print(f_acc)
        # Total Forces
        force_vec = f_roll_vec + f_g_path + f_acc
        # print(force_vec)
        self.motor(force_vec, dt)
        return self.kinetic

    def motor(self, force_vector, dt):
        # Motor generates force along path of travel, need to check direction
        unit_path = self.path / np.linalg.norm(self.path)
        unit_force = force_vector / np.linalg.norm(force_vector)
        if np.linalg.norm(unit_force - unit_path) < 1.0e-10:
            rps = (np.linalg.norm(self.path) / dt) / (self.wheel_dia * math.pi)
            dv_wheel = rps - self.wheel_speed
            f_inertia_wheel = self.wheel_moment * (dv_wheel / dt)
            # print(f_inertia_wheel)
            force_per_motor = np.linalg.norm(force_vector) / 6
            torque_per_motor = (force_per_motor * self.wheel_dia / 2) + f_inertia_wheel
            # Calculate Electrical Motor Parameters
            # Assuming speed control with PWM-like method
            i_avg = (torque_per_motor / self.gear_ratio) / self.motor_tc
            duty_cycle_min = i_avg / self.motor_peak_current
            #print(duty_cycle_min)
            if duty_cycle_min >= 1.0:
                duty_cycle_min = 1.0
            v_min = duty_cycle_min * self.battery_v
            rps_loss = ((self.motor_st_grad * (torque_per_motor/self.gear_ratio)) / (self.gear_ratio * 60))
            v_avg = ((rps + rps_loss) * 60 * self.gear_ratio) / self.motor_kv
            if v_avg >= self.battery_v:
                v_avg = self.battery_v
                # print("Unable to travel at requested speed")
            if v_min > v_avg:
                v_avg = v_min
            #print(v_avg)
            """print("Using", i_avg * 6, " amps at ", v_avg, " Volts, for a total power of: ", i_avg * 6 * v_avg,
                  " Watts.")"""
            self.battery -= (v_avg * i_avg * 6) * dt
        #else:
            #print("misaligned forces")




if __name__ == "__main__":
    w_d_in = 9
    full_size_specs = {
        'mass': 50,
        'wheel_moment': (0.5*1*((0.5*w_d_in*0.0254)**2)),  # 1/2MR^2    R = 1/2 * Diameter(in)*(in_to_m)
        'wheel_dia': (w_d_in*0.0254),  # 9 in to m
        'motor_st_grad': 693,  # Speed Torque Gradient RPM/Nm, reduction in RPM from torque
        'mech_loss': 0.05,  # should be low without a gearbox
        'rolling_resistance': 0.2,
        'battery_ah': 18,
        'battery_v': 21.6,
        'motor_kv': 80,
        'motor_tc': 0.13,      # 0.083 * 40 = 3.32 Nm   max 0.083 * 80 = 6.64 Nm
        'motor_rated_current': 4.8,
        'motor_peak_current': 12,
        'gear_ratio': 8
    }
    test_rover = Rover(full_size_specs)
    new_position = np.array([0, 2, 0.2])  # approximately 6 degrees
    kinetic = test_rover.main_physics(new_position, 1)
    # print(kinetic)


    # print(test_rover.max_speed_rot, test_rover.max_speed, test_rover.max_torque, test_rover.max_force)

    """velocity_vector = np.array([0, 10, 20])
    kinetic = 0.5 * 50 * (velocity_vector**2)
    print(kinetic)"""



