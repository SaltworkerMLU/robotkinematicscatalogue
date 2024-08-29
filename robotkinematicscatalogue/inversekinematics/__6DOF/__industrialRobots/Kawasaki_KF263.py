from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Kawasaki_KF263(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          710-710,    np.pi/2     ],
                        [   -np.pi/2,   250,        0,          -np.pi/2    ],
                        [   0,          1100,       0,          0           ],
                        [   -np.pi/2,   200,        1300,       -np.pi/2    ],
                        [np.deg2rad(60),0,          85,         0           ],
                        [-np.deg2rad(60),0,          97.5,       np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [150, 110, 90, 720, 720, 410]
        self.jointMin = [-150, -60, -80, -720, -720, -410]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, -1, 1, 1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper