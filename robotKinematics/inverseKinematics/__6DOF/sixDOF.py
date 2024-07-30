from robotKinematics.src import *
from robotKinematics.forwardKinematics import *
import numpy as np

class sixDOF(forwardKinematics):
    """
    # This class does 6DOF forward kinematics.
    """
    def FK(self,joint : list, start : int = 1, ender : int = 6):
        """
        Do numeric forward kinematics for 6DOF robots.

        Note how @start and @end indicate which of the DHM-parameters are to be used.

        EXAMPLES: 
        - start = 1 and end = 1 will make function return transform matrix T01
        - start = 1 and end = 2 will make function return transform matrix T02
        
        Parameters
        ---

        joint : float[6]
            The joint values of a robot used for forward kinematics. 
            
            Using joint value "None" is interpreted as joint value equal 0
        
        start : int
            The first joint to do forward kinematics of. 
        
        end : int
            The last joint to do forward kinematics of. 

        """
        
        # Array contains sum of DHM-parameter and joint θ
        IOtheta = np.zeros(ender)

        for i in range(start-1, ender):
            # Having joint elements equal "None" implies joint equal 0
            if joint[i] == None:
                IOtheta[i] = self.theta[i] # + 0
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
                    TBW = np.dot(TBW, angleSetConventions.transformMatrix([0, 0, 0, 0, 0, -np.rad2deg(joint[i-j])], "XYZ"))

        return TBW