from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class ABB_IRB_580_16(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=angleSetConventions.transformMatrix([0, 0, 0, 90, 0, 0], "ZYX")):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          630,        0           ],
                        [   -np.pi/2,   0,          0,          -np.pi/2    ],
                        [   0,          1000,       0,          0           ],
                        [   -np.pi/2,   170.5,      1408.404,   -np.pi/2    ],
                        [np.deg2rad(35),0,          0,          0           ],
                        [-np.deg2rad(35),0,          211.596,    np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [150, 75, 145, 360, 360, 720]
        self.jointMin = [-150, -60, -145, -360, -360, -720]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 1, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper