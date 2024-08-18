from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Motoman_EPX2050(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          600-600,    0           ],
                        [   -np.pi/2,   300,        0,          -np.pi/2    ],
                        [   0,          850,        0,          0           ],
                        [   -np.pi/2,   310,        850,        -np.pi/2    ],
                        [np.deg2rad(60),0,          89.951172,  0           ],
                        [-np.deg2rad(60),0,          110.024414, 0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [90, 100, 95, 360, 360, 360]
        self.jointMin = [-90, -50, -73, -360, -360, -360]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1, -1, -1, -1, -1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper