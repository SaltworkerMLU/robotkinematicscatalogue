from robotKinematics.forwardKinematics import *
from robotKinematics.angleSetConventions import *

class palletizingRobot(forwardKinematics):
    """
    # Robot kinematics of a palletizing robot
    """
    
    def FK(self, joints, start=1, ender=5):
        # Robot joints are allocated as so on palletizing robots
        joint = [joints[0], joints[1], joints[2], None, joints[3]]

        # Array contains sum of DHM-parameter and joint θ
        IOtheta = np.zeros(5)

        for i in range(start-1, ender):
            if joint[i] == None:
                IOtheta[i] = self.theta[i]
            else:
                IOtheta[i] = self.theta[i] + self.inv_joint[i] * joint[i]

        TBW = np.eye(4)
        for i in range(start-1, ender):
            # The equation of temp implies *modified* DH-parameters are used.
            # Another equation must be used for DH-parameters (go look it up).
            temp = np.array([
                [np.cos(IOtheta[i]), -np.sin(IOtheta[i]), 0, self.a[i]],
                [np.sin(IOtheta[i]) * np.cos(self.alpha[i]), np.cos(IOtheta[i]) * np.cos(self.alpha[i]), -np.sin(self.alpha[i]), -np.sin(self.alpha[i]) * self.d[i]],
                [np.sin(IOtheta[i]) * np.sin(self.alpha[i]), np.cos(IOtheta[i]) * np.sin(self.alpha[i]), np.cos(self.alpha[i]), np.cos(self.alpha[i]) * self.d[i]],
                [0, 0, 0, 1]
            ])
            TBW = np.dot(TBW, temp)

            # Nullify rotation of prior joint(s)
            if self.null_joint[i] != 0:
                for j in range(1, self.null_joint[i]+1):
                    TBW = np.dot(TBW, angleSetConventions.transformMatrix([0, 0, 0, 0, 0, -self.inv_joint[i-j] * np.rad2deg(joint[i-j])], "XYZ"))

        return TBW

    def IK(self, TBW):
        TB5 = TBW @ np.linalg.inv(self.T6W) 

        Joint = np.zeros([2,4])

        # Theta1
        Joint[:, 0] = self.inv_joint[0] * np.arctan2( float(TB5[1,3]), float(TB5[0,3]) )

        # Theta4 = Theta1 - R_z
        # NOTE: Changing equation of theta4 will make it more buggy than before. You have been warned
        # Determine which direction to make 180 degree turn
        tDOF = R.from_matrix(TB5[:3, :3]).as_euler("XYZ",degrees=True) # Get intrinsic XYZ angle set
        Joint[:,3] = self.inv_joint[0] * Joint[0,0] + tDOF[2] * np.pi / 180 + self.theta[4]
        """if tDOF[2] > 0: 
           Joint[:,3] = -Joint[0,0] - self.inv_joint[3] * tDOF[2] * np.pi / 180 - np.pi
        else:
           Joint[:,3] = -Joint[0,0] - self.inv_joint[3] * tDOF[2] * np.pi / 180 + np.pi"""

        T01 = self.FK(Joint[0, :], 1, 1)
        T12 = angleSetConventions.transformMatrix([self.a[1], 0, 0, 0, -90, -90 ], "ZYX")
        T45 = self.FK(Joint[0, :], 5, 5)

        T24 = np.linalg.inv(T01 @ T12) @ TBW @ np.linalg.inv(T45 @ self.T6W)

        hyp = np.linalg.norm( [float(T24[0,3]), float(T24[1,3])]) # T2W[0,3]**2 + T2W[1,3]**2 # The transform matrix T2W works in the XY-plane

        # Theta2 - calculated using 
        phi1 = np.arccos( float( T24[0,3] ) / hyp * 1)
        phi2 = np.arccos(( hyp**2 + self.a[2]**2 - self.a[3]**2 ) / ( 2 * self.a[2] *  hyp))

        Joint[:,1] = [phi1 - phi2 - np.pi/2 - self.theta[1], phi1 + phi2 - np.pi/2 - self.theta[1]]

        # Theta3
        phi3 = np.arccos(( -hyp**2 + self.a[2]**2 + self.a[3]**2 ) / ( 2 * self.a[2] *  self.a[3]))

        Joint[:,2] = [-self.inv_joint[2] * phi3 + self.theta[4], self.inv_joint[2] * phi3 + self.theta[4]]

        # Does theta_2 cancel out in joint 3
        if self.null_joint[2] != 0:
            Joint[:,2] += [self.inv_joint[2] * Joint[0,1] + self.theta[1], self.inv_joint[2] * Joint[1,1] + self.theta[1]]
        
        Joint = Joint * 180 / np.pi

        # Add any additional solutions by makeing a full revolution of 360 degrees whereever possible
        for i in range(len(Joint[:,0])):
            # If said joint can make a full revolution of 360 degrees, in either direction, do it
                if Joint[i, 3] + 360 < self.jointMax[3]:
                    Joint[i, 3] += 360
                    Joint = np.vstack([Joint, Joint[i]])
                    Joint[i, 3] -= 360 
                elif Joint[i, 3] - 360 > self.jointMin[3]:
                    Joint[i, 3] -= 360 
                    Joint = np.vstack([Joint, Joint[i]])
                    Joint[i, 3] += 360

        # Remove all solutions outside of joint range
        for i in range(len(Joint)):
            for j in range(len(Joint[0])): # for each available joint
                if (Joint[i,j] > self.jointMax[j] or Joint[i,j] < self.jointMin[j]):
                    Joint[i] = None
        
        # Delete solutions containing "None"
        Joint = Joint[~np.isnan(Joint).any(axis=1)]

        return Joint
    
    