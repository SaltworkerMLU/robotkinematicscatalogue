from robotkinematicscatalogue.inversekinematics.__6DOF.sixDOF import *

class collaborativeRobot(sixDOF):
    """
    # Robot kinematics of any collaborative robot, e.g. Universal robots (UR)
    """

    def IK(self, TBW) -> np.matrix:
        T06s = np.linalg.inv(self.TB0) @ TBW @ np.linalg.inv(self.T6W) # T06 = TBO^-1 * TBW * T6W^-1

        T00 = np.eye(4)
        T00[2, 3] = self.d[0]
        T06 = np.linalg.inv(T00) @ T06s

        Joint = np.zeros([8, 6]) # 8 unique solutions; 6 joints
        for i in range(8):
            #Theta1
            P05 = T06 @ np.array([0, 0, -self.d[5], 1])
            phi1 = np.arctan2(P05[1], P05[0])
            phi2 = np.arccos(self.d[3] / np.sqrt(P05[0]**2 + P05[1]**2))

            #phi1 = np.arctan2(T06[1,3], T06[0,3])
            #phi2 = np.arccos((self.d[3] + self.d[5]) / np.sqrt(T06[1,3]**2 + T06[0,3]**2))

            #print(90 + (phi1 + phi2) * 180 / np.pi)

            # Eliminate all solutions containing complex values
            if np.iscomplex(phi2):
                Joint[i, :] = [None] * 6
                continue

            phi = [-np.real(phi2), np.real(phi2)]
            Joint[i, 0] = self.theta[0] + phi1 + phi[i//4 % 2] + np.pi/2
            Joint[i, 0] *= self.inv_joint[0]

            #Theta5
            T01 = np.linalg.inv(self.TB0) @ np.linalg.inv(T00) @self.FK(Joint[i, :], 1, 1) @ np.linalg.inv(self.T6W)
            T16 = np.linalg.inv(T01) @ T06
            #phi1 = np.arccos((-T16[1, 3] - self.d[3]) / self.d[5])
            # print(np.arccos(-T16[1,2]) * 180 / np.pi)
            phi1 = np.pi + np.arccos(T16[1,2])

            # Eliminate all solutions containing complex values
            if np.iscomplex(phi1):
                Joint[i, :] = [None] * 6
                continue

            phi = [np.real(phi1), -np.real(phi1)]

            Joint[i, 4] = self.theta[4] + phi[i//2 % 2]
            
            # Theta6
            Joint[i, 5] = self.theta[4] + self.theta[5] + np.arctan2(-T16[1,1] / np.sin(Joint[i, 4]), T16[1,0] / np.sin(Joint[i, 4])) # + np.pi

            Joint[i, 5] *= self.inv_joint[5]

            if Joint[i,5] > np.pi:
                Joint[i,5] = Joint[i,5] - 2 * np.pi

            # Get T14 using acquired joints
            T45 = np.linalg.inv(self.TB0) @ self.FK(Joint[i, :], 5, 5) @ np.linalg.inv(self.T6W)
            T56 = np.linalg.inv(self.TB0) @ self.FK(Joint[i, :], 6, 6) @ np.linalg.inv(self.T6W)
            T14 = T16 @ np.linalg.inv(T45 @ T56)

            #Theta3
            T14xz = np.linalg.norm([T14[0, 3], T14[2, 3]])
            phi1 = np.arccos((T14xz**2 - self.a[2]**2 - self.a[3]**2) / (2 * self.a[2] * self.a[3]))
            
            # Eliminate all solutions containing complex values
            if np.iscomplex(phi1):
                Joint[i, :] = [None] * 6
                continue

            phi = [np.real(phi1), -np.real(phi1)]
            Joint[i, 2] = self.theta[2] + phi[i % 2]

            #theta2
            phi1 = np.arctan2(-T14[2, 3], -T14[0, 3])
            phi2 = np.arcsin(self.a[3] * np.sin(self.inv_joint[2] * Joint[i, 2]) / T14xz)
            Joint[i, 1] = -self.theta[1] + phi1 - phi2 + np.pi
            Joint[i, 1] *= self.inv_joint[1] # Don't worry about it. It works

            #Theta4
            T12 = np.linalg.inv(self.TB0) @ self.FK(Joint[i, :], 2, 2) @ np.linalg.inv(self.T6W)
            T23 = np.linalg.inv(self.TB0) @ self.FK(Joint[i, :], 3, 3) @ np.linalg.inv(self.T6W)
            T34 = np.linalg.inv(T12 @ T23) @ T14
            Joint[i, 3] = -self.theta[3] + np.arctan2(T34[1, 0], T34[0, 0])
            Joint[i, 3] *= self.inv_joint[3]

            Joint[i, 4] *= self.inv_joint[4] # Thank you python very cool

        Joint = np.degrees(Joint)
        for i in range(len(Joint)):
            if Joint[i,0] == None:
                Joint[i,:] = None

        solutions = np.zeros([len(Joint[:,0]) * 2**6, 6])

        # Make solution set assuming joint range is [-360, 360] for each joint
        for i in range(2**6):
            temp = Joint.copy()
            config = [i % 2, i//2 % 2, i//4 % 2, i//8 % 2, i // 16 % 2, i // 32 % 2]
            for j in range(len(Joint)):
                for k in range(6):
                    if config[k] == 0:
                        continue
                    if temp[j, k] > 0:
                        temp[j, k] -= 360
                    else:
                        temp[j, k] += 360
                solutions[(i * Joint.shape[0] + j), :] = temp[j, :]
        
        # Eliminate solutions outside of joint range
        for i in range(len(solutions)): # len(solutions[:,0]):
            for j in range(len(solutions[0])): 
                if solutions[i, j] < self.jointMin[j] or solutions[i, j] > self.jointMax[j]:
                    solutions[i,:] = None
                    break # go to next solution
        
        # Delete solutions containing "None"
        solutions = solutions[~np.isnan(solutions).any(axis=1)]
        
        # Remove identical solutions (and sort solutions in a proper order)
        #solutions = np.unique(solutions, axis=0)

        # Get incorrect IK solutions (for testing purposes)
        wrongSolutions = np.empty([1,6])
        for i in range(len(solutions)):
            temp = self.FK(np.deg2rad(solutions[i]))
            comparison = TBW.astype(np.float16) == temp.astype(np.float16)
            if comparison.all() == False:
                wrongSolutions = np.vstack([wrongSolutions, solutions[i]])

        wrongSolutions = np.delete(wrongSolutions, (0), axis=0)

        return solutions, wrongSolutions