from robotkinematicscatalogue.inversekinematics.__6DOF.collaborativeRobot import *

class RoboDK_RDK_COBOT_1200(collaborativeRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          201,        0           ],
                        [   np.pi/2,    0,          0,          np.pi/2     ],
                        [   0,          590,        0,          0           ],
                        [   0,          560,        175,        -np.pi/2    ],
                        [   -np.pi/2,   0,          140,        0           ],
                        [   np.pi/2,    0,          117,        -np.pi/2    ]])
 
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [360, 180, 160, 180, 180, 360]
        self.jointMin = [-360, -180, -160, -180, -180, -360]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1, 1, 1, -1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper