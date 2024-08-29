from robotkinematicscatalogue.forwardKinematics import *
from robotkinematicscatalogue.angleSetConventions import *

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

        TBW = self.TB0
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

        TBW = np.dot(TBW, self.T6W)

        return TBW

    def IK(self, TBW):
        TB5s = np.linalg.inv(self.TB0) @ TBW @ np.linalg.inv(self.T6W) 

        T00 = np.eye(4)
        T00[2, 3] = self.d[0]

        TB5 = np.linalg.inv(T00) @ TB5s

        Joint = np.zeros([2,4])

        # Theta1
        Joint[:, 0] = self.inv_joint[0] * np.arctan2( float(TB5[1,3]), float(TB5[0,3]) )

        # Theta4 = Theta1 - R_z
        # NOTE: Changing equation of theta4 will make it more buggy than before. You have been warned
        # Determine which direction to make 180 degree turn
        tDOF = R.from_matrix(TB5[:3, :3]).as_euler("XYZ",degrees=True) # Get intrinsic XYZ angle set
        Joint[:,3] = self.inv_joint[0] * Joint[0,0] + tDOF[2] * np.pi / 180 + self.theta[4]
        Joint[:,3] *= self.inv_joint[4]
        """if tDOF[2] > 0: 
           Joint[:,3] = -Joint[0,0] - self.inv_joint[3] * tDOF[2] * np.pi / 180 - np.pi
        else:
           Joint[:,3] = -Joint[0,0] - self.inv_joint[3] * tDOF[2] * np.pi / 180 + np.pi"""

        T01 = np.linalg.inv(self.TB0) @ self.FK(Joint[0, :], 1, 1) @ np.linalg.inv(self.T6W) 
        T12 = angleSetConventions.transformMatrix([self.a[1], 0, 0, 0, -90, -90 ], "ZYX")
        T45 = np.linalg.inv(self.TB0) @ self.FK(Joint[0, :], 5, 5) @ np.linalg.inv(self.T6W) 

        T24 = np.linalg.inv(T01 @ T12) @ TBW @ np.linalg.inv(T45)

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
        print(Joint)
        solutions = np.zeros([len(Joint[:,0]) * 2**4, 4])

        # Make solution set assuming joint range is [-360, 360] for each joint
        for i in range(2**4):
            temp = Joint.copy()
            config = [i % 2, i//2 % 2, i//4 % 2, i//8 % 2, i // 16 % 2, i // 32 % 2]
            for j in range(len(Joint)):
                for k in range(4):
                    if config[k] == 0:
                        continue
                    if temp[j, k] > 0:
                        temp[j, k] -= 360
                    else:
                        temp[j, k] += 360
                solutions[(i * Joint.shape[0] + j), :] = temp[j, :]
        
        # Remove all solutions outside of joint range
        for i in range(len(solutions)):
            for j in range(len(solutions[0])): # for each available joint
                if (solutions[i,j] > self.jointMax[j] or solutions[i,j] < self.jointMin[j]):
                    solutions[i,:] = None
                    break # go to next solution
        
        # Delete solutions containing "None"
        solutions = solutions[~np.isnan(solutions).any(axis=1)]

        # Remove identical solutions (and sort solutions in a proper order)
        solutions = np.unique(solutions, axis=0)

        # Get incorrect IK solutions (for testing purposes)
        wrongSolutions = np.empty([1,4])
        for i in range(len(solutions)):
            temp = self.FK(np.deg2rad(solutions[i]))
            comparison = TBW.astype(np.float16) == temp.astype(np.float16)
            if comparison.all() == False:
                wrongSolutions = np.vstack([wrongSolutions, solutions[i]])

        wrongSolutions = np.delete(wrongSolutions, (0), axis=0)

        return solutions, wrongSolutions