#import important stuff
import numpy as np

#Compute the Denavit-Hartenberg transformation matrix
def dh_transformation(alpha, a, d, theta):
    """
    theta (float): Angle in radians about the previous z-axis to the common normal
    d (float): Offset along the previous z to the common normal
    a (float): Length of the common normal (the distance between z_i−1 and z_i axes)
    alpha (float): Angle in radians about the common normal to the next x-axis
    """
    return np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        			[np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        			[0, np.sin(alpha), np.cos(alpha), d],
        			[0, 0, 0, 1]])
    
#Compute the homogeneous transformation for the kinematic chain for planar test
def kinematic_chain(dh_parameters):
	# dh_parameters: Denavit-Hartenberg parameters for each joint

    H = np.identity(4)
    for params in dh_parameters:
        H = np.dot(H, dh_transformation(*params))
    return H
    
# Compute the homogeneous transformation for the kinematic chain for case 1 and 2
def kinematic_chain2(dh_parameters, theta):
    # dh_parameters: Denavit-Hartenberg parameters for each joint

    H = np.identity(4)
    for params, angle in zip(dh_parameters, theta):
        params_with_theta = list(params)
        params_with_theta[-1] = angle  # Replace theta with the provided angle
        H = np.dot(H, dh_transformation(*params_with_theta))
    return H
    
#Extract the position components (x, y, z) from a homogeneous transformation matrix
def get_pos(H):
	# H: Homogeneous transformation matrix
    return H[:3, 3]

#Extract the roll, pitch, and yaw angles from a homogeneous transformation matrix
def get_rot(H):
	# H: Homogeneous transformation matrix
    
    roll = np.arctan2(H[2, 1], H[2, 2])
    pitch = np.arctan2(-H[2, 0], np.sqrt(H[2, 1]**2 + H[2, 2]**2))
    yaw = np.arctan2(H[1, 0], H[0, 0])
    return roll, pitch, yaw
