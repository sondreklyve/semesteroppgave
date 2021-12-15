import numpy as np

phi = 43  # Initial angle in degrees
phi = np.deg2rad(phi)  # Converts phi to radians so it works with numpy functions
v0 = 0  # Initial velocity
a_list = [0]  # Initialize list for acceleration
theta_list = [phi]  # Initialize list of angles
v_list = [v0]  # Initialize list for velocity
theta_k_friction_list = []  # Initialize list of angles after kinetic friction

R = 0.5  # Radius of track
r = 0.02  # Radius of object
g = 9.81  # Gravitational acceleration
c = 1  # Constant determining moment of inertia
mu_s = 0.25  # Static frictional coefficient
mu_k = 0.20  # Kinetic frictional coefficient

delta_t = 0.0001

d_mu = 0.1
d_g = 0.01
d_theta = np.deg2rad(3)
d_R = 0.5
d_r = 0.05
d_v = 0.01


def k_uncertainty(R, r, g, mu, v, theta):
    global d_mu, d_g, d_theta, d_R, d_r, d_v
    del_mu = -g*np.sin(theta) + v**2/(R+r)
    del_g = np.sin(theta) - mu*np.sin(theta) + v**2/(R+r)
    del_phi = g*np.cos(theta) - mu*np.cos(theta)
    del_R = -g*v**2/(R+r)**2
    del_r = -g*v**2/(R+r)**2
    del_v = 2*v/(R+r)
    return ((del_mu*d_mu)**2 + (del_g*d_g)**2 + (del_phi*d_theta)**2 + (del_R*d_R)**2 + (del_r*d_r)**2 + (del_v*d_v)**2)


def s_uncertainty(g, theta):
    global d_g, d_theta
    del_g = np.sin(theta)/(1+c)
    del_theta = g*np.cos(theta)/(1+c)
    return ((del_g*d_g)**2 + (del_theta*d_theta)**2)**.5


def normal_acceleration(theta, v1, g, R):
    """ Function that finds normal force of the object from the track """
    return g * np.cos(theta) - v1 ** 2 / (R + r)


def friction_threshold(g, theta, v, r, R, mu, a_theta):
    """ Returns whether the normal force is too small, and the object starts slipping"""
    if a_theta * c <= mu * (g * np.cos(theta) - v ** 2 / (R + r)):
        return False  # Object does not slip, hence static friction coefficient
    return True  # Object does slip, hence kinetic friction coefficient


i = 0  # Initialize counting variable
check = False  # Will become True if kinetic friction kicks in to not go back to static friction

while normal_acceleration(theta_list[i], v_list[i], g, R) > 0:  # Loop breaks when normal force exceeds 0

    if friction_threshold(g, theta_list[i], v_list[i], R, r, mu_s, a_list[i]) or check:
        # Acceleration is calculated using kinetic friction coefficient
        a_list.append(g * np.sin(theta_list[i]) - (g * np.cos(theta_list[i]) - v_list[i] ** 2 / (R + r)) * mu_k)
        theta_k_friction_list.append(theta_list[i])  # Adds the angle to this list if there is kinetic friction
        check = True

        d_v = (delta_t**2 * k_uncertainty(R,r,g,mu_k,v_list[i],theta_list[i])**2 + d_v**2)**.5
        d_theta = (delta_t**2 * d_v**2 + d_theta**2)**.5
    else:
        # Acceleration is calculated assuming pure roll, since the frictional force is sufficient
        a_list.append(9.81 * np.sin(theta_list[i]) / (1 + c))

        d_v = (delta_t**2 * s_uncertainty(g, theta_list[i])**2 + d_v**2)**.5
        d_theta = (delta_t**2 * d_v**2 + d_theta**2)**.5

    theta_list.append(theta_list[i] + v_list[i] * delta_t / (R + r))  # Using Euler's method to find next angle
    v_list.append(v_list[i] + a_list[i] * delta_t)  # Using Euler's method to find next velocity

    i += 1  # Increment counting variable


numerical_theta_crit = np.rad2deg(theta_list[-1])  # Gets last angle in the list, and converts it to degrees

print(f"Critical angle using Euler's method: {numerical_theta_crit}°")

if theta_k_friction_list:  # Only runs if there is kinetic friction
    theta_k_friction = np.rad2deg(theta_k_friction_list[0])  # Gets the first angle after kinetic friction, and converts it to degrees
    print(f"Angle when kinetic friction starts using Euler's method: {theta_k_friction}°")

print(np.rad2deg(d_theta))
