import numpy as np

phi = 0.00001  # Initial angle
v0 = 0  # Initial velocity
theta_list = [phi]  # Initialize list of angles
v_list = [v0]  # Initialize list of velocities

R = 0.5  # Radius of track
m = 0.168  # Mass of object
g = 9.81  # Gravitational acceleration
delta_t = 0.0001


def normal_force(theta, v1, m, g, R):
    """ Function that finds normal force of the object from the track """
    return m * g * np.cos(theta) - m * v1 ** 2 / R


i = 0  # Initialize counting variable
while normal_force(theta_list[i], v_list[i], m, g, R) > 0:  # Loop breaks when normal force exceeds 0
    theta_list.append(theta_list[i] + 2 * v_list[i] * delta_t)  # Using Euler's method to find next angle
    v_list.append(v_list[i] + 9.81 * np.sin(theta_list[i]) * delta_t)  # Using Euler's method to find next velocity
    i += 1  # Increment counting variable

numerical_theta_crit = np.rad2deg(theta_list[-1])  # Gets last angle in the list, and converts it to degrees
analytical_theta_crit = np.rad2deg(
    np.arccos(2 / 3 * np.cos(phi)))  # Finds critical angle analytically, and converts it to degrees

print(f"Critical angle using Euler's method: {numerical_theta_crit}°")
print(f"Critical angle using analytical solution: {analytical_theta_crit}°")
print(f"Error using Euler's method: {abs(analytical_theta_crit - numerical_theta_crit)}°")
