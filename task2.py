import numpy as np


def cubic_coefficients(theta_0, theta_f, t_0, t_f, vel_0=0, vel_f=0):
    """
    Computes the cubic polynomial coefficients given initial and final
    positions and velocities for a joint.

    Parameters:
    theta_0: Initial joint angle
    theta_f: Final joint angle
    t_0: Initial time
    t_f: Final time
    vel_0: Initial velocity (default = 0)
    vel_f: Final velocity (default = 0)

    Returns:
    Coefficients a0, a1, a2, a3 of the cubic polynomial
    """
    # Setup the matrix of equations
    A = np.array([[1, t_0, t_0 ** 2, t_0 ** 3],
                  [0, 1, 2 * t_0, 3 * t_0 ** 2],
                  [1, t_f, t_f ** 2, t_f ** 3],
                  [0, 1, 2 * t_f, 3 * t_f ** 2]])

    # Setup the boundary conditions for position and velocity
    B = np.array([theta_0, vel_0, theta_f, vel_f])

    # Solve for coefficients
    coefficients = np.linalg.solve(A, B)

    return coefficients


def cubic_trajectory(coefficients, t):
    """
    Evaluates the cubic trajectory at time t.

    Parameters:
    coefficients: The cubic polynomial coefficients [a0, a1, a2, a3]
    t: The time at which to evaluate the trajectory

    Returns:
    The joint angle at time t
    """
    a0, a1, a2, a3 = coefficients
    return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3

import matplotlib.pyplot as plt

# Time parameters
t_0 = 0  # Initial time
t_f = 10  # Final time
times = np.linspace(t_0, t_f, 100)  # Time points for plotting

# Initial and final positions and velocities for each joint
theta_0_q1, theta_f_q1 = np.pi/2, np.pi/4  # Joint 1
theta_0_q2, theta_f_q2 = 0, np.pi/4  # Joint 2
theta_0_q3, theta_f_q3 = 0, -np.pi/4  # Joint 3

# Compute cubic coefficients for each joint
coeff_q1 = cubic_coefficients(theta_0_q1, theta_f_q1, t_0, t_f)
coeff_q2 = cubic_coefficients(theta_0_q2, theta_f_q2, t_0, t_f)
coeff_q3 = cubic_coefficients(theta_0_q3, theta_f_q3, t_0, t_f)

# Evaluate the trajectories for each joint
q1_traj = [cubic_trajectory(coeff_q1, t) for t in times]
q2_traj = [cubic_trajectory(coeff_q2, t) for t in times]
q3_traj = [cubic_trajectory(coeff_q3, t) for t in times]

# Plot the joint trajectories
plt.figure()
plt.plot(times, q1_traj, label='q1 (Joint 1)')
plt.plot(times, q2_traj, label='q2 (Joint 2)')
plt.plot(times, q3_traj, label='q3 (Joint 3)')
plt.title('Cubic Trajectories for Joints q1, q2, q3')
plt.xlabel('Time (s)')
plt.ylabel('Joint Angle (rad)')
plt.legend()
plt.grid(True)
plt.show()
