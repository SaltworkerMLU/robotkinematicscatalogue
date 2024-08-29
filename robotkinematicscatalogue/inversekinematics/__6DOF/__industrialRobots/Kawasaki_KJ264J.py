from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Kawasaki_KJ264J(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          909-909,    np.pi/2     ],
                        [   -np.pi/2,   140,        0,          -np.pi/2    ],
                        [   0,          1100,       0,          0           ],
                        [   -np.pi/2,   0,          1400,       -np.pi/2    ],
                        [np.deg2rad(60),0,          100.004,    0           ],
                        [-np.deg2rad(60),0,          104.998,    np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [120, 130, 90, 720, 720, 410]
        self.jointMin = [-120, -80, -65, -720, -720, -410]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, -1, 1, 1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper