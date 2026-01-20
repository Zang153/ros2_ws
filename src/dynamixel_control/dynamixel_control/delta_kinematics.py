import numpy as np
import math

class DeltaKinematics:
    def __init__(self, rod_b=0.1, rod_ee=0.2, r_b=0.074577, r_ee=0.02495):
        '''
        Configuration for the Delta robot
        rod_b = length of the link connected to the base (Upper arm)
        rod_ee = length of the link connected to the end-effector (Lower arm)
        r_b   = radius of the base (distance from center to pin joints)
        r_ee  = radius of the end effector (distance from center to universal joints)
        '''
        self.rod_b = rod_b
        self.rod_ee = rod_ee
        self.r_b = r_b
        self.r_ee = r_ee
        self.alpha = np.array([0, 120, 240])

    def sind(self, x):
        return np.sin(np.deg2rad(x))

    def cosd(self, x):
        return np.cos(np.deg2rad(x))
        
    def tand(self, x):
        return np.tan(np.deg2rad(x))

    def ik(self, _3d_pose):
        '''
        Inverse Kinematics
        Input: _3d_pose [x, y, z] (numpy array or list)
        Output: theta [theta1, theta2, theta3] in degrees
        '''
        # Frame in Delta manipulator is different from the Frame in the UAV. Need to transform.
        rod_ee = self.rod_ee
        rod_b = self.rod_b
        r_ee = self.r_ee
        r_b = self.r_b 
        alpha = self.alpha 
        
        # Ensure input is numpy array
        _3d_pose = np.array(_3d_pose)

        # Frame Transform: Rotate -90 degrees around Z? 
        # [[0, 1, 0], [-1, 0, 0], [0, 0, 1]] * [x, y, z]^T = [y, -x, z]^T
        mat = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        pos = np.dot(mat, _3d_pose)
        
        [x0, y0, z0] = pos	
        
        theta = [0.0, 0.0, 0.0]

        for i in [0, 1, 2]:
            # Rotate coordinate system for each arm
            x = x0 * self.cosd(alpha[i]) + y0 * self.sind(alpha[i])
            y = -x0 * self.sind(alpha[i]) + y0 * self.cosd(alpha[i])
            z = z0

            # Coordinate of the joint on the end-effector (in the rotated frame)
            # Actually the math below calculates the intersection of sphere (arm length) 
            # and circle (motor rotation plane).
            
            # This logic is ported directly from the reference code.
            
            # E1_pos seems to be the position of the EE joint relative to the arm plane?
            # In the reference code:
            # ee_pos = [x, y, z] (rotated)
            # E1_pos = ee_pos + [0, -r_ee, 0]  <-- shift by EE radius
            # F1_pos[i] = [0, -r_b, 0] <-- Base joint location in rotated frame?
            
            # Let's trust the reference implementation for the geometric derivation.
            
            # E1_pos is likely the EE joint position in the "i-th arm frame" 
            # BUT offset by r_ee.
            
            # The code variables:
            # _x0, _y0, _z0 are coordinates of the EE joint relative to the arm plane center?
            
            _x0 = x
            _y0 = y - r_ee # Shifted by EE radius
            _z0 = z
            
            _yf = -r_b # Base joint y-coordinate in rotated frame (since it's on the radius)
            
            # Intersection of sphere centered at (_x0, _y0, _z0) with radius rod_ee
            # and circle centered at (0, _yf, 0) with radius rod_b in YZ plane?
            
            # Solving for theta:
            # (y_elbow - _yf)^2 + (z_elbow - 0)^2 = rod_b^2
            # (y_elbow - _y0)^2 + (z_elbow - _z0)^2 + (0 - _x0)^2 = rod_ee^2
            
            # The reference code derivation:
            c1 = (_x0**2 + _y0**2 + _z0**2 + rod_b**2 - rod_ee**2 - _yf**2) / (2 * _z0)
            c2 = (_yf - _y0) / _z0
            c3 = -(c1 + c2 * _yf)**2 + (c2**2 + 1) * rod_b**2

            if c3 < 0:
                # print(f"Non existing point for arm {i}")
                return -1

            J1_y = (_yf - c1 * c2 - c3**0.5) / (c2**2 + 1)
            J1_z = c1 + c2 * J1_y
            F1_y = -r_b

            # Calculate angle
            theta[i] = math.atan(-J1_z / (F1_y - J1_y)) * 180.0 / np.pi
            
        return np.array(theta) * (-1) # The definition of +/- is opposite in the reference.
