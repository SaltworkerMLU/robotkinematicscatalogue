from robotkinematicscatalogue.inversekinematics.__6DOF.collaborativeRobot import *

class Efort_GR680(collaborativeRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          598.6,      0           ],
                        [   -np.pi/2,   210,        0,          np.pi/2     ],
                        [   0,          1000,       0,          -np.deg2rad(85.619)],
                        [   0,          1505.399,   0,          -np.deg2rad(184.381)],
                        [   -np.pi/2,   0,          136.5,      -np.pi/2    ],
                        [   np.pi/2,    0,          101.75,     0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [175, 80, 125, 360, 360, 360]
        self.jointMin = [-175, -150, -80, -360, -360, -360]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, -1, -1, 1, 1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper