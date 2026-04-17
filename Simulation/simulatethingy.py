import numpy as np
import matplotlib.pyplot as plt
from rover import Rover


def generate_surface(distance, steps):
    A = 4
    B = 0.05
    base = np.linspace(0, distance, steps)
    low_f = A * np.sin(np.linspace(0, B * distance, steps))
    A2 = 3
    B2 = 0.2
    X2 = distance/10
    m_f = -A2 * np.sin(np.linspace(0+X2, (B2 * distance) +X2, steps))
    A3 = 1
    B3 = 0.6
    h_f = A3 * np.sin(np.linspace(0, B3 * distance, steps))
    A4 = 10
    B4 = 0.03
    X4 = distance/25
    low_f2 = A4 * np.maximum(0, np.sin(np.linspace(0+X4, (B4 * distance)+X4, steps)))
    combined = np.maximum(-A/2, low_f+m_f+h_f+low_f2)

    return base, combined/3


def plot_surface(base, surface):
    fix, ax = plt.subplots()
    ax.plot(base, surface)
    ax.set(xlim=(0, 25), ylim=(0, 25))
    plt.show()
    print(surface)

def find_target(base, surface, distance):
    try:
        index = np.where(base >= distance)[0][0]
    except IndexError:
        index = len(base) -1
    y = base[index]
    height = surface[index]
    return np.array([0, y, height]), index

def sim_loop(base, surface, rover, v, dt):
    origin = np.array([0, base[0], surface[0]])
    rover.position = origin
    #print(rover.position)
    i = 0
    initial_energy = rover.battery
    while i < len(base)-1:
        # print(i)
        new_position, i = find_target(base, surface, ((rover.position[1]) + (v*dt)))
        # print(new_position)
        rover.main_physics(new_position, dt)
        rover.position = new_position
        # print(rover.battery)
    print("Used Energy: ", round((initial_energy-rover.battery)/3600, 2), " Wh")
if __name__ == "__main__":
    w_d_in = 10
    full_size_specs = {
        'mass': 50,
        'wheel_moment': (0.5*1*((0.5*w_d_in*0.0254)**2)),  # 1/2MR^2    R = 1/2 * Diameter(in)*(in_to_m)
        'wheel_dia': (w_d_in*0.0254),  # 9 in to m
        'motor_st_grad': 240,  # Speed Torque Gradient RPM/Nm, reduction in RPM from torque
        'mech_loss': 0.05,  # should be low without a gearbox
        'rolling_resistance': 0.2,
        'battery_ah': 12,
        'battery_v': 48,
        'motor_kv': 30,
        'motor_tc': 0.32,      # 0.083 * 40 = 3.32 Nm   max 0.083 * 80 = 6.64 Nm
        'motor_rated_current': 4.8,
        'motor_peak_current': 12,
        'gear_ratio': 8
    }
    test_rover = Rover(full_size_specs)
    base, surface = generate_surface(2000, 20000)
    # plot_surface(base, surface)
    sim_loop(base, surface, test_rover, 2, 0.1)
