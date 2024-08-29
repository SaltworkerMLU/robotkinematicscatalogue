from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Leantec_LA2000_8_C(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          517,        0           ],
                        [   -np.pi/2,   150,        0,          -np.pi/2    ],
                        [   0,          760,        0,          0           ],
                        [   -np.pi/2,   140,        1080.65,    0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          121.5,      np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [170, 150, 75, 180, 130, 350]
        self.jointMin = [-170, -100, -162, -180, -130, -350]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper