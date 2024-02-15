#import stuff including robot model
import numpy as np
from robot_model import dh_transformation, kinematic_chain, kinematic_chain2, get_pos, get_rot


#planar mainpulator testing
def test_planar_manipulator():
    # Denavit-Hartenberg parameters for the two-link planar manipulator
    a1 = 1
    a2 = 1
    theta1 = np.pi/2
    theta2 = np.pi/2
    
    # Define DH parameters for each joint
    dh_params = [[theta1, 0, a1, 0],
        		 [theta2, 0, a2, 0]]
    
    # Compute kinematics
    H = kinematic_chain(dh_params)
    
    # Extract position and orientation
    pos = get_pos(H)
    rot = get_rot(H)
    
    # Print results
    print("Two-Link Planar Manipulator:")
    print(f"Position (x, y, z): {pos}")
    print(f"Orientation (roll, pitch, yaw): {rot}\n")

#ur5e testing, case 1
def test_ur5e_case1():
    # DH parameters for UR5e robot - Case 1
    dh_params = [[np.pi/2, 0, 0.1625, 0],
        		[0, -0.425, 0, 0],
        		[0, -0.39225, 0, 0],
        		[np.pi/2, 0, 0.1333,0],
        		[-np.pi/2, 0, 0.0997, 0],
        		[0, 0, 0.0996, 0]]
    
    # Joint angles for Case 1
    theta = [0, 0, 0, 0, 0, 0]
    
    # Compute kinematics
    H = kinematic_chain2(dh_params, theta)
    
    # Extract position and orientation
    pos = get_pos(H)
    rot = get_rot(H)
    
    # Print results
    print("UR5e Robot - Case 1:")
    print(f"Position (x, y, z): {pos}")
    print(f"Orientation (roll, pitch, yaw): {rot}\n")

#ur5e testing, case 2
def test_ur5e_case2():
    # DH parameters for UR5e robot - Case 2
    dh_params = [[np.pi/2, 0, 0.1625, 0],
        		[0, -0.425, 0, 0],
        		[0, -0.39225, 0, 0],
        		[np.pi/2, 0, 0.1333,0],
        		[-np.pi/2, 0, 0.0997, 0],
        		[0, 0, 0.0996, 0]]
    
    # Joint angles for Case 2
    theta = [0, -np.pi/2, 0, 0, 0, 0]
    
    # Compute kinematics
    H = kinematic_chain2(dh_params,theta)
    
    # Extract position and orientation
    pos = get_pos(H)
    rot = get_rot(H)
    
    # Print results
    print("UR5e Robot - Case 2:")
    print(f"Position (x, y, z): {pos}")
    print(f"Orientation (roll, pitch, yaw): {rot}\n")
    

if __name__ == "__main__":
    # Test the two-link planar manipulator
    test_planar_manipulator()
    
    # Test UR5e robot - Case 1
    test_ur5e_case1()
    
    # Test UR5e robot - Case 2
    test_ur5e_case2()
