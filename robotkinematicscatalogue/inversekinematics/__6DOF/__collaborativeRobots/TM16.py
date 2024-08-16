from robotkinematicscatalogue.inversekinematics.__6DOF.collaborativeRobot import *

class TM16(collaborativeRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          165.2,      0           ],
                        [   np.pi/2,    0,          0,          np.pi/2     ],
                        [   0,          500,        0,          0           ],
                        [   0,          311.5,      156.3,      -np.pi/2    ],
                        [   -np.pi/2,   0,          106,        0           ],
                        [   np.pi/2,    0,          113,        np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [270, 180, 155, 180, 180, 270]
        self.jointMin = [-270, -180, -155, -180, -180, -270]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, -1, -1, -1, 1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper