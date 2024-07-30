from robotKinematics.src import *
from robotKinematics.inverseKinematics.__6DOF.collaborativeRobot import *

class TM5_700(collaborativeRobot):
    
    def __init__(self, gripper=np.diag([-1, -1, 1, 1])):
                                                        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          145.2,      0           ],
                        [   np.pi/2,    0,          0,          np.pi/2     ],
                        [   0,          329,        0,          0           ],
                        [   0,          311,        122.3,      -np.pi/2    ],
                        [   -np.pi/2,   0,          106.5,      0           ],
                        [   np.pi/2,    0,          114.15,     np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [360, 180, 155, 180, 180, 270]
        self.jointMin = [-360, -180, -155, -180, -180, -270]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, -1, -1, -1, 1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # Predefined transform matrices used for inverse kinematics
        self.TB0 = np.eye(4)
        self.TB0[2, 3] = self.d[0]
        self.T6W = gripper