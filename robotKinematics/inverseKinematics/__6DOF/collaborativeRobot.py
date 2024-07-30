from robotKinematics.src import *
from robotKinematics.inverseKinematics.__6DOF.sixDOF import *
import numpy as np

class collaborativeRobot(sixDOF):
    """
    # Robot kinematics of any collaborative robot, e.g. Universal robots (UR)
    """

    def IK(self, TBW) -> np.matrix:
        T06 = np.linalg.inv(self.TB0) @ TBW @ np.linalg.inv(self.T6W) # T06 = TBO^-1 * TBW * T6W^-1

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
            T01 = np.linalg.inv(self.TB0) @ self.FK(Joint[i, :], 1, 1)
            T16 = np.linalg.inv(T01) @ T06
            #phi1 = np.arccos((-T16[1, 3] - self.d[3]) / self.d[5])
            # print(np.arccos(-T16[1,2]) * 180 / np.pi)
            phi1 = np.arccos(-T16[1,2])

            # Eliminate all solutions containing complex values
            if np.iscomplex(phi1):
                Joint[i, :] = [None] * 6
                continue

            phi = [np.real(phi1), -np.real(phi1)]
            Joint[i, 4] = self.theta[4] + phi[i//2 % 2]
            Joint[i, 4] *= self.inv_joint[4]

            # Theta6
            Joint[i, 5] = self.theta[5] + np.arctan2(T16[1,1] / np.sin(Joint[i, 4]), -T16[1,0] / np.sin(Joint[i, 4]))

            Joint[i, 5] *= self.inv_joint[5]

            if Joint[i,5] > np.pi:
                Joint[i,5] = Joint[i,5] - 2 * np.pi

            # Get T14 using acquired joints
            T45 = self.FK(Joint[i, :], 5, 5)
            T56 = self.FK(Joint[i, :], 6, 6) @ np.linalg.inv(self.T6W)
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
            phi1 = np.arctan2(-T14[2, 3], -self.inv_joint[2] * T14[0, 3])
            phi2 = np.arcsin(self.a[3] * np.sin(Joint[i, 2]) / T14xz)
            Joint[i, 1] = -self.theta[1] + phi1 - phi2 + np.pi # NOTE: self.theta[1] = 0 with UR30
            Joint[i, 1] *= self.inv_joint[2] # Don't worry about it. It works

            #Theta4
            T12 = self.FK(Joint[i, :], 2, 2)
            T23 = self.FK(Joint[i, :], 3, 3)
            T34 = np.linalg.inv(T12 @ T23) @ T14
            Joint[i, 3] = -self.theta[3] + np.arctan2(T34[1, 0], T34[0, 0])

        Joint = np.degrees(Joint)
        for i in range(len(Joint)):
            if Joint[i,0] == None:
                Joint[i,:] = None

        solutions = np.zeros([len(Joint[:,0]) * 64, 6])

        # Make solution set assuming joint range is [-360, 360] for each joint
        for i in range(64):
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
        
        solutions = solutions[~np.isnan(solutions).any(axis=1)]

        print(f"\nTotal amount of IK solutions: {len(solutions)}")

        return solutions