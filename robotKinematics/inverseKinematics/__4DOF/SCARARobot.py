from robotKinematics.forwardKinematics import *

class SCARARobot(forwardKinematics):
    """
    # Robot kinematics of any SCARA robot
    The SCARA robot is a 4DOF RRPR robot known for its effectiveness in industrial pick&place tasks.

    This class takes care of forward- and inverse kinematics of the SCARA robot

    ## DHM-table

    Where underscore indicates joint value. E.g. θ_1 is a joint value while d1 is a constant

    __i__ | α_{i-1} | a_{i-1} | d_i | θ_i
    --- | --- | --- | --- | ---
    **1** | 0 | 0 | 0 | θ_1
    **2** | 0 | a1 | 0 | θ_2
    **3** | 0 | a2 | d_3 | 0
    **4** | 180 | 0 | 0 | θ_4

    ---
    
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

        TBW = np.eye(4)
        for i in range(start-1, ender):
            temp = np.array([
                [np.cos(IOtheta[i]), -np.sin(IOtheta[i]), 0, self.a[i]],
                [np.sin(IOtheta[i]) * np.cos(self.alpha[i]), np.cos(IOtheta[i]) * np.cos(self.alpha[i]), -np.sin(self.alpha[i]), -np.sin(self.alpha[i]) * IOd[i]],
                [np.sin(IOtheta[i]) * np.sin(self.alpha[i]), np.cos(IOtheta[i]) * np.sin(self.alpha[i]), np.cos(self.alpha[i]), np.cos(self.alpha[i]) * IOd[i]],
                [0, 0, 0, 1]
            ])
            TBW = np.dot(TBW, temp)

        return TBW

    def IK(self, TBW):
        T04 = np.linalg.inv(self.TB0) @ TBW @ np.linalg.inv(self.T4W)

        Joint = np.zeros([2, 4]) # 2 unique solutions; 4 joints

        # Start off with getting the translational joint defined as "self.translationalJoint"
        if self.translationalJoint == 4:
            Joint[:,self.translationalJoint-1] = [ abs(TBW[2,3]) - self.d[self.translationalJoint-1] ]
        else:
            Joint[:,self.translationalJoint-1] = [ TBW[2,3] - self.d[self.translationalJoint-1] ]

        # theta2
        if self.translationalJoint >= 3:
            phi = np.arccos( (T04[0,3]**2 + T04[1,3]**2 - self.a[1]**2 - self.a[2]**2) / (2*self.a[1]*self.a[2]) )
            Joint[:, 1] = [phi, -phi]
        else:
            phi = np.arccos( (T04[0,3]**2 + T04[1,3]**2 - self.a[2]**2 - self.a[3]**2) / (2*self.a[2]*self.a[3]) )
            Joint[:, 2] = [phi, -phi]

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
            Joint[:,1] = np.arctan2( -C1*C4 - C2*C3, C2*C4 - C1*C3 )
        else:
            Joint[:,0] = np.arctan2( -C1*C4 - C2*C3, C2*C4 - C1*C3 )

        # theta4 = Theta1 - Rz

        # get R_z by isolating for said value in rotation matrix -> acos() and you will find ...
        # ... it has two possible values which are accounted for by checking if Rz is positive or negative
        tDOF = R.from_matrix(T04[:3, :3]).as_euler("ZYX",degrees=True) # Get intrinsic XYZ angle set
        if self.translationalJoint == 4:
            Joint[:,2] = - ( Joint[:,0] + Joint[:,1] - tDOF[0] * np.pi / 180) + np.pi*2
        if self.translationalJoint == 3:
            Joint[:,3] = Joint[:,0] + Joint[:,1] - tDOF[0] * np.pi / 180
        if self.translationalJoint == 2:
            Joint[:,3] = Joint[:,0] + Joint[:,2] - tDOF[0] * np.pi / 180
        if self.translationalJoint == 1:
            Joint[:,3] = Joint[:,1] + Joint[:,2] - tDOF[0] * np.pi / 180
        #if tDOF[2] > 0: # Is positive value
        #    Joint[:,3] = Joint[:,0] + Joint[:,1] + np.arccos(T04[0,0]) 
        #else:  # Is negative value
        #    Joint[:,3] = Joint[:,0] + Joint[:,1] - np.arccos(T04[0,0])
        
        Joint = Joint * 180 / np.pi
        Joint[:,self.translationalJoint-1] = np.deg2rad(Joint[:,self.translationalJoint-1])

        # Add any additional solutions by makeing a full revolution of 360 degrees whereever possible
        if self.translationalJoint == 4:
            lastRotation = 2
        else:
            lastRotation = 3
            
        for i in range(len(Joint[:,0])):
            # If said joint can make a full revolution of 360 degrees, in either direction, do it
                if Joint[i, lastRotation] + 360 < self.jointMax[lastRotation]:
                    Joint[i, lastRotation] += 360
                    Joint = np.vstack([Joint, Joint[i]])
                    Joint[i, lastRotation] -= 360 
                elif Joint[i, lastRotation] - 360 > self.jointMin[lastRotation]:
                    Joint[i, lastRotation] -= 360 
                    Joint = np.vstack([Joint, Joint[i]])
                    Joint[i, lastRotation] += 360

        # Remove all solutions outside of joint range
        for i in range(len(Joint)):
            for j in range(len(Joint[0])): # for each available joint
                if (Joint[i,j] > self.jointMax[j] or Joint[i,j] < self.jointMin[j]):
                    Joint[i] = None
        
        # Delete solutions containing "None"
        Joint = Joint[~np.isnan(Joint).any(axis=1)]

        print(f"\nTotal amount of IK solutions: {len(Joint)}")

        return Joint