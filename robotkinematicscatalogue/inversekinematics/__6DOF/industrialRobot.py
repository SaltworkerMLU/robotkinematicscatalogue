from robotkinematicscatalogue.inversekinematics.__6DOF.sixDOF import *

class industrialRobot(sixDOF):
    """
    # Robot kinematics of 6DOF industrial robots.
    """

    # https://web.archive.org/web/20180725190826id_/http://dergipark.gov.tr/download/article-file/311007
    def IK(self, TBW):
        # Alert user of sources of error in IK calculations
        if self.d[1] != 0 or self.d[4] != 0 or self.a[5] != 0:
            print("ATTENTION: d_2, d_5, and a_5 have not been accounted for in Inverse Kinematics calculations.")
        
        for i in range(len(self.null_joint)):
            if self.null_joint[i] != 0:
                print("ATTENTION: Nullified joints cause a fluxuating joint range. Some solutions may be out of joint range.")

        T06s = np.linalg.inv(self.TB0) @ TBW @ np.linalg.inv(self.T6W)

        T00 = np.eye(4)
        T66 = np.eye(4)
        T00[2, 3] = self.d[0]
        T66[2, 3] = self.d[5]
        T06 = np.linalg.inv(T00) @ T06s @ np.linalg.inv(T66)


        Joint = np.zeros([8, 6]) # 8 unique solutions; 6 joints
        for i in range(8):   
            # Theta1
            phi = [np.arctan2(T06[1,3], T06[0,3]), np.arctan2(-T06[1,3], -T06[0,3])]

            """if np.real(phi) < 0:
                phi = [np.real(phi), np.real(phi)+np.pi] # add 180 degrees if initial theta value negative
            else:
                phi = [np.real(phi), np.real(phi)-np.pi]""" # subtract 180 degrees if initial theta value negative

            Joint[i, 0] = self.theta[0] + phi[i//4 % 2] # + np.arctan2(T06[1,3], self.a[1])/2
            Joint[i, 0] *= self.inv_joint[0]

            # Acquiring Theta2 and Theta3 - each have 4 possible solutions adjacent with one another
            T01 = np.linalg.inv(self.TB0) @ np.linalg.inv(T00) @ self.FK(Joint[i, :], 1, 1) @ np.linalg.inv(self.T6W)
            T16 = np.linalg.inv(T01) @ T06

            s = T16[2, 3]
            r = T16[0, 3] - self.a[1]

            A = np.linalg.norm([r, s])
            B = self.a[2]
            C = np.linalg.norm([ self.d[3], self.a[3] ])

            # Theta3
            phi1 = np.real ( np.arccos ( (A**2 - B**2 - C**2) / (2 * B * C ) ) )
            phi2 = np.arctan(self.a[3] / self.d[3])

            # Theta2
            phi3 = np.arctan2(s, r)
            phi4 = np.arctan2( C * np.sin(phi1), B + C * np.cos(phi1) )
            #phi4 = np.real ( np.arccos ( -(- A**2 - B**2 + C**2) / (2 * B * A ) ) )

            if i < 2 or i >= 6:
                Joint[i, 1] = - self.theta[1] - phi4 - phi3
                Joint[i, 2] = self.theta[1] + phi1 + phi2
            else:
                Joint[i, 1] = - self.theta[1] + phi4 - phi3
                Joint[i, 2] = self.theta[1] - phi1 + phi2

            #Joint[i, 1] *= self.inv_joint[1]
            #Joint[i, 2] *= self.inv_joint[2]

            # Check if theta2 above 180 degrees
            if Joint[i, 1] > np.pi:
                Joint[i, 1] -= np.pi*2
            if Joint[i, 1] < -np.pi:
                Joint[i, 1] += np.pi*2
            
            # Check if theta3 above 180 degrees
            if Joint[i, 2] > np.pi:
                Joint[i, 2] -= np.pi*2
            if Joint[i, 2] < -np.pi:
                Joint[i, 2] += np.pi*2

            Joint[i, 1] = self.inv_joint[1] * Joint[i, 1]
            Joint[i, 2] = self.inv_joint[2] * (Joint[i, 2] - np.pi/2 - self.theta[1] - self.theta[2])

            if self.null_joint[2] == 1:
                Joint[i, 2] += self.inv_joint[2] * Joint[i, 1]

            # Acquiring Theta4, Theta5 & Theta6
            T12 = np.linalg.inv(self.TB0) @ self.FK(Joint[i, :], 2, 2) @ np.linalg.inv(self.T6W)
            T23 = np.linalg.inv(self.TB0) @ self.FK(Joint[i, :], 3, 3) @ np.linalg.inv(self.T6W)
            T03 = T01 @ T12 @ T23
            T36 = np.linalg.inv(T03) @ T06

            phi = np.real( np.arccos(T36[1,2]) )

            phi = [phi, -phi]

            Joint[i, 4] = phi[i % 2] - self.theta[4]

            Joint[i, 3] = np.arctan2(T36[2,2] * np.sin(Joint[i,4] + self.theta[4]), -T36[0,2] * np.sin(Joint[i, 4] + self.theta[4])) - self.theta[3]
            Joint[i, 5] = np.arctan2(T36[1,1] * np.sin(Joint[i,4] + self.theta[4]), -T36[1,0] * np.sin(Joint[i, 4] + self.theta[4])) + np.pi - self.theta[5]

            Joint[i, 3] = self.inv_joint[3] * Joint[i, 3]
            Joint[i, 4] = self.inv_joint[4] * Joint[i, 4]
            Joint[i, 5] = self.inv_joint[5] * Joint[i, 5]
        
        Joint = np.degrees(Joint)
        Joint = Joint[~np.isnan(Joint).any(axis=1)]
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