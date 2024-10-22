import numpy as np


def inverse_kinematics_with_orientation(X, Y, theta, L1, L2, L3):
    # Calculate intermediate target point (before end effector)
    x_wrist = X - L3 * np.cos(theta)
    y_wrist = Y - L3 * np.sin(theta)

    D = (x_wrist ** 2 + y_wrist ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)

    if abs(D) > 1:
        raise ValueError("Target is outside of the reachable workspace.")

    # Compute q2 (elbow angle)
    q2 = np.arctan2(np.sqrt(1 - D ** 2), D)  # Elbow-up solution

    # Compute q1 (shoulder angle)
    q1 = np.arctan2(y_wrist, x_wrist) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    q3 = theta - q1 - q2

    return q1, q2, q3

def forward_kinematics(q1, q2, q3):
    """
    Calculate the (x, y) position of the end-effector based on the joint angles.
    """
    x = (L1 * np.cos(q1) +
         L2 * np.cos(q1 + q2) +
         L3 * np.cos(q1 + q2 + q3))
    y = (L1 * np.sin(q1) +
         L2 * np.sin(q1 + q2) +
         L3 * np.sin(q1 + q2 + q3))
    return x, y

# Example usage
L1 = 0.675*2  # Length of the first link
L2 = 0.675*2  # Length of the second link
L3 = 3.167-0.675*4  # Length of the third link
theta = np.pi / 4  # orientation angle of the end-effector

X, Y = 0, 3
q1, q2, q3 = inverse_kinematics_with_orientation(X, Y, theta, L1, L2, L3)
print("q1:", np.degrees(q1), "degrees")
print("q2:", np.degrees(q2), "degrees")
print("q3:", np.degrees(q3), "degrees")

print(forward_kinematics(q1, q2, q3))