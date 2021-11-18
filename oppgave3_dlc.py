import numpy as np
import matplotlib.pyplot as plt

phi = 1  # Initial angle in degrees 
phi = np.deg2rad(phi)  # Converts phi to radians so it works with numpy functions
v0 = 0  # Initial velocity
theta_list = [phi]  # Initialize list of angles
v_list = [v0]  # Initialize list of velocities
theta_k_friction_list = []  # Initialize list of angles after kinetic friction
omega_0 = 0  # Initial angular velocity
omega_list = [omega_0]  # Intialize list of angular velocities

R = 0.5  # Radius of track
r = 0.1  # Radius of object
g = 9.81  # Gravitational acceleration
c = .5  # Constant determining moment of inertia
mu_s = 0  # Static frictional coefficient
mu_k = 0  # Kinetic frictional coefficient
m = 1  # Mass of object
a0 = 9.81 * np.sin(phi) / (1 + c)  # Initial acceleration
a_list = [a0]  # Initial radial acceleration

E_list = [m * (g * (R+r) * np.cos(phi) + .5 * v0 ** 2 * (1+c))]  # Initialize list of mechanical energy

delta_t = 0.00001


def normal_acceleration(theta, v1, g, R):
    """ Function that finds normal force of the object from the track """
    return g * np.cos(theta) - v1 ** 2 / (R + r)


def friction_threshold(g, theta, v, r, mu, c, a_theta):
    """ Returns whether the normal force is too small, and the object starts slipping"""
    if a_theta * c <= mu * (g * np.cos(theta) - v ** 2 / r):
        return False  # Object does not slip, hence static friction coefficient
    return True  # Object does slip, hence kinetic friction coefficient


i = 0  # Initialize counting variable
check = False  # Will become True if kinetic friction kicks in to not go back to static friction

while normal_acceleration(theta_list[i], v_list[i], g, R) > 0:  # Loop breaks when normal force exceeds 0

    if friction_threshold(g, theta_list[i], v_list[i], (R + r), mu_s, c, a_list[i]) or check:
        # Acceleration is calculated using kinetic friction coefficient
        a_list.append(g * np.sin(theta_list[i]) - (g * np.cos(theta_list[i]) - (v_list[i]) ** 2 / (R+r)) * mu_k)
        theta_k_friction_list.append(theta_list[i])  # Adds the angle to this list if there is kinetic friction
        check = True
        omega_list.append(omega_list[i] + mu_k/(c*r**2) * (g*r*np.cos(theta_list[i] - v_list[i]**2) * delta_t))  # Adds angular velocity using Euler's method
    else:
        # Acceleration is calculated assuming pure roll, since the frictional force is sufficient
        a_list.append(9.81 * np.sin(theta_list[i]) / (1 + c))
        omega_list.append(v_list[i]/r)  # Adds angular velocity using v=omega/r because of pure roll

    theta_list.append(theta_list[i] + v_list[i] * delta_t / (R + r))  # Using Euler's method to find next angle
    v_list.append((v_list[i] + a_list[i] * delta_t))  # Using Euler's method to find next velocity
    E_list.append(m*g*(R+r)*np.cos(theta_list[i]) + .5*m*v_list[i]**2 + .5*c*m*r**2 *omega_list[i]**2)  # Finding the mechanical energy

    i += 1  # Increment counting variable


numerical_theta_crit = np.rad2deg(theta_list[-1])  # Gets last angle in the list, and converts it to degrees

print(f"Critical angle using Euler's method: {numerical_theta_crit}°")

if theta_k_friction_list:  # Only runs if there is kinetic friction
    theta_k_friction = np.rad2deg(theta_k_friction_list[0])  # Gets the first angle after kinetic friction, and converts it to degrees
    print(f"Angle when kinetic friction starts using Euler's method: {theta_k_friction}°")


t = len(v_list) * delta_t  # Finds the total time in seconds before the object loses contact
x_v = np.linspace(0,t,len(v_list))
x_a = np.linspace(0,t,len(a_list))
x_E = np.linspace(0,t,len(E_list))


plt.figure(0)
plt.plot(x_v, v_list)
plt.title('Hastighet')
plt.xlabel('tid: s')
plt.ylabel('hastighet: m/s')

plt.figure(1)
plt.plot(x_a, a_list, 'r')
plt.title('Akselerasjon')
plt.xlabel('tid: s')
plt.ylabel('akselerasjon: m/s^2')

plt.figure(2)
plt.plot(x_E, E_list, 'y')
plt.title('Total mekanisk energi')
plt.xlabel('tid: s')
plt.ylabel('Energi: J')

plt.show()
