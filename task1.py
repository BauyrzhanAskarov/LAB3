# Define the forward kinematics of the 3 DoF planar robot

import numpy as np
import matplotlib.pyplot as plt

# Step 1: Define link lengths based on measurements (in meters)
L0 = 0
L1 = 0.675*2  # Length of the first link
L2 = 0.675*2  # Length of the second link
L3 = 3.167-0.675*4  # Length of the third link


# Step 2: Define the forward kinematics function for the 3 DoF planar robot
def forward_kinematics(q1, q2, q3):
    """
    Compute the forward kinematics for a 3 DoF planar robot.

    q1, q2, q3: Joint angles in radians
    Returns: (x, y) position of the end-effector
    """
    # Compute the position of the end-effector
    x = L0 + L1 * np.cos(q1) + L2 * np.cos(q1 + q2) + L3 * np.cos(q1 + q2 + q3)
    y = L0 + L1 * np.sin(q1) + L2 * np.sin(q1 + q2) + L3 * np.sin(q1 + q2 + q3)
    return x, y


# Step 3: Implement the function to compute the workspace
def plot_workspace():
    """
    Plot the workspace of the 3 DoF planar robot.
    """
    # Define the range of joint angles (in radians)
    q1_range = np.linspace(-np.pi / 2, np.pi / 2, 100)
    q2_range = np.linspace(-np.pi / 2, np.pi / 2, 100)
    q3_range = np.linspace(-np.pi / 2, np.pi / 2, 100)

    # Initialize lists to store the x and y positions of the end-effector
    x_positions = []
    y_positions = []

    # Compute the Cartesian positions for the entire range of joint angles
    for q1 in q1_range:
        for q2 in q2_range:
            for q3 in q3_range:
                x, y = forward_kinematics(q1, q2, q3)
                x_positions.append(x)
                y_positions.append(y)

    # Step 4: Plot the workspace
    plt.figure(figsize=(8, 8))
    plt.plot(x_positions, y_positions, 'b.', markersize=1)
    plt.title('Workspace of 3 DoF Planar Robot')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True)
    plt.axis('equal')
    plt.show()


# Call the function to plot the workspace
plot_workspace()
