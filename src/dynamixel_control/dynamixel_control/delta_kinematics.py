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

    def fk(self, theta):
        '''
        Forward Kinematics
        Input: theta [theta1, theta2, theta3] in degrees
        Output: _3d_pose [x, y, z] (numpy array)
        '''
        rod_b = self.rod_b
        rod_ee = self.rod_ee
        
        # Ensure input is numpy array
        theta = np.array(theta)
        
        # The IK implementation multiplies result by -1 at the end.
        # So FK input should likely be multiplied by -1 to match the math?
        # Let's check IK: returns theta * (-1)
        # So if we pass the physical angles (which are likely the negative of math angles),
        # we should flip them back.
        # However, usually FK takes the values as returned by IK (or read from motors).
        # If motor returns X, and IK returns X, then FK(X) should be Position.
        # If IK returns -Theta_math, then FK should take -Theta_math.
        # The math in `fk` reference implementation takes `theta`.
        # Let's see if reference `fk` handles the sign.
        # Reference `fk` just uses `theta`.
        # Reference `ik` returns `theta * (-1)`.
        # So if I pass the output of IK into FK, I should probably flip the sign first
        # OR `fk` math expects the "math angles".
        # Let's assume `fk` expects "math angles".
        # Since IK returns "Physical angles" (Math * -1),
        # FK should take "Physical angles" and convert to "Math angles" ( * -1)
        # OR I should check if `fk` logic matches `ik` logic signs.
        # In IK: theta[i] = atan(...)
        # In FK: y1 = -(t + rod_b*cosd(theta1))
        # If I use `ik` output directly, let's verify.
        
        # Let's multiply by -1 to revert the IK's last step, assuming IK output is what we feed in.
        theta = theta * (-1)

        theta1 = theta[0]
        theta2 = theta[1]
        theta3 = theta[2]

        side_ee = 2 / self.tand(30) * self.r_ee
        side_b  = 2 / self.tand(30) * self.r_b

        t = (side_b - side_ee) * self.tand(30) / 2

        y1 = -(t + rod_b * self.cosd(theta1))
        z1 = -rod_b * self.sind(theta1)

        y2 = (t + rod_b * self.cosd(theta2)) * self.sind(30)
        x2 = y2 * self.tand(60)
        z2 = -rod_b * self.sind(theta2)

        y3 = (t + rod_b * self.cosd(theta3)) * self.sind(30)
        x3 = -y3 * self.tand(60)
        z3 = -rod_b * self.sind(theta3)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1**2 + z1**2
        w2 = x2**2 + y2**2 + z2**2
        w3 = x3**2 + y3**2 + z3**2

        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2

        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2

        a = a1**2 + a2**2 + dnm**2
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm**2)
        c = (b2 - y1 * dnm)**2 + b1**2 + dnm**2 * (z1**2 - rod_ee**2)

        d = b**2 - 4 * a * c
        if d < 0:
            return -1

        z0 = -0.5 * (b + d**0.5) / a
        x0 = (a1 * z0 + b1) / dnm
        y0 = (a2 * z0 + b2) / dnm
        
        # Now [x0, y0, z0] is in Delta Frame.
        # We need to transform back to World Frame (inverse of IK transform).
        # IK: World -> Delta: [y, -x, z]
        # So Delta [x0, y0, z0] corresponds to [y_w, -x_w, z_w]
        # x0 = y_w  => y_w = x0
        # y0 = -x_w => x_w = -y0
        # z0 = z_w  => z_w = z0
        
        return np.array([-y0, x0, z0])

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
