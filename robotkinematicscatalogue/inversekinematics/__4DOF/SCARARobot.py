from robotkinematicscatalogue.forwardKinematics import *

class SCARARobot(forwardKinematics):
    """
    # Robot kinematics of any SCARA robot
    """

    def FK(self, joint, start=1, ender=4):
        IOtheta = np.zeros(ender)
        IOd = np.zeros(ender)

        IOtheta = [self.theta[0] + self.inv_joint[0] * joint[0], 
                   self.theta[1] + self.inv_joint[1] * joint[1], 
                   self.theta[2] + self.inv_joint[2] * joint[2], 
                   self.theta[3] + self.inv_joint[3] * joint[3]]
        IOd = [self.d[0], self.d[1], self.d[2], self.d[3]]

        IOtheta[self.translationalJoint-1] -= self.inv_joint[self.translationalJoint-1] * joint[self.translationalJoint-1]
        IOd[self.translationalJoint-1] += self.inv_joint[self.translationalJoint-1] * np.rad2deg(joint[self.translationalJoint-1])

        TBW = self.TB0
        for i in range(start-1, ender):
            temp = np.array([
                [np.cos(IOtheta[i]), -np.sin(IOtheta[i]), 0, self.a[i]],
                [np.sin(IOtheta[i]) * np.cos(self.alpha[i]), np.cos(IOtheta[i]) * np.cos(self.alpha[i]), -np.sin(self.alpha[i]), -np.sin(self.alpha[i]) * IOd[i]],
                [np.sin(IOtheta[i]) * np.sin(self.alpha[i]), np.cos(IOtheta[i]) * np.sin(self.alpha[i]), np.cos(self.alpha[i]), np.cos(self.alpha[i]) * IOd[i]],
                [0, 0, 0, 1]
            ])
            TBW = np.dot(TBW, temp)

        TBW = np.dot(TBW, self.T6W)

        return TBW

    def IK(self, TBW):
        T04s = np.linalg.inv(self.TB0) @ TBW @ np.linalg.inv(self.T6W)

        T00 = T44 = np.eye(4)
        T00[2, 3] = self.d[0]
        T44[2, 3] = self.d[3]

        T04 = np.linalg.inv(T00) @ T04s @ np.linalg.inv(T44)

        Joint = np.zeros([2, 4]) # 2 unique solutions; 4 joints

        # Start off with getting the translational joint defined as "self.translationalJoint"
        if self.translationalJoint == 4:
            Joint[:,self.translationalJoint-1] = [ abs(TBW[2,3]) - self.d[self.translationalJoint-1] ]
        else:
            Joint[:,self.translationalJoint-1] = [ TBW[2,3] - self.d[self.translationalJoint-1] ]
        
        Joint[:,self.translationalJoint-1] *= self.inv_joint[self.translationalJoint-1]

        # theta2
        if self.translationalJoint >= 3:
            phi = np.arccos( (T04[0,3]**2 + T04[1,3]**2 - self.a[1]**2 - self.a[2]**2) / (2*self.a[1]*self.a[2]) )
            Joint[:, 1] = [phi, -phi]
            Joint[:, 1] *= self.inv_joint[1]
        else:
            phi = np.arccos( (T04[0,3]**2 + T04[1,3]**2 - self.a[2]**2 - self.a[3]**2) / (2*self.a[2]*self.a[3]) )
            Joint[:, 2] = [phi, -phi]
            Joint[:, 2] *= self.inv_joint[2]

        #theta1
        if self.translationalJoint <= 2:
            C1 = self.a[2] + self.a[3]*np.cos(Joint[:, 2])
            C2 = -self.a[3]*np.sin(Joint[:, 2])
        else:
            C1 = self.a[1] + self.a[2]*np.cos(Joint[:, 1])
            C2 = -self.a[2]*np.sin(Joint[:, 1])
 
        C3 = -T04[0,3]
        C4 = -T04[1,3]
        if self.translationalJoint == 1:
            Joint[:,1] = self.inv_joint[1] * np.arctan2( -C1*C4 - C2*C3, C2*C4 - C1*C3 )
        else:
            Joint[:,0] = self.inv_joint[0] * np.arctan2( -C1*C4 - C2*C3, C2*C4 - C1*C3 )

        # theta4 = Theta1 - Rz

        # get R_z by isolating for said value in rotation matrix -> acos() and you will find ...
        # ... it has two possible values which are accounted for by checking if Rz is positive or negative
        tDOF = R.from_matrix(T04[:3, :3]).as_euler("ZYX",degrees=True) # Get intrinsic XYZ angle set
        if self.translationalJoint == 4:
            Joint[:,2] = - ( Joint[:,0] + Joint[:,1] - tDOF[0] * np.pi / 180) + np.pi*2
        if self.translationalJoint == 3:
            Joint[:,3] = self.inv_joint[0] * Joint[:,0] + self.inv_joint[1] * Joint[:,1] - tDOF[0] * np.pi / 180
        if self.translationalJoint == 2:
            Joint[:,3] = self.inv_joint[0] * Joint[:,0] + self.inv_joint[2] * Joint[:,2] - tDOF[0] * np.pi / 180
        if self.translationalJoint == 1:
            Joint[:,3] = self.inv_joint[1] * Joint[:,1] + self.inv_joint[2] * Joint[:,2] - tDOF[0] * np.pi / 180
        #if tDOF[2] > 0: # Is positive value
        #    Joint[:,3] = Joint[:,0] + Joint[:,1] + np.arccos(T04[0,0]) 
        #else:  # Is negative value
        #    Joint[:,3] = Joint[:,0] + Joint[:,1] - np.arccos(T04[0,0])
        Joint[:,3] *= self.inv_joint[3]
        
        Joint = Joint * 180 / np.pi
        Joint[:,self.translationalJoint-1] = np.deg2rad(Joint[:,self.translationalJoint-1])

        solutions = np.zeros([len(Joint[:,0]) * 2**4, 4])

        # Make solution set assuming joint range is [-360, 360] for each joint
        for i in range(2**4):
            temp = Joint.copy()
            config = [i % 2, i//2 % 2, i//4 % 2, i//8 % 2, i // 16 % 2, i // 32 % 2]
            for j in range(len(Joint)):
                for k in range(4):
                    if self.translationalJoint-1 == k:
                        continue
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
        wrongSolutions = np.empty([1, 4])
        for i in range(len(solutions)):
            temp = self.FK(np.deg2rad(solutions[i]))
            comparison = TBW.astype(np.float16) == temp.astype(np.float16)
            if comparison.all() == False:
                wrongSolutions = np.vstack([wrongSolutions, solutions[i]])

        wrongSolutions = np.delete(wrongSolutions, (0), axis=0)

        return solutions, wrongSolutions