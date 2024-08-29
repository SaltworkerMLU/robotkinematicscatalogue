from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Staubli_RX60B(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          550-550,    0           ],
                        [   -np.pi/2,   150,        0,          -np.pi/2    ],
                        [   0,          825,        0,          -np.pi/2    ],
                        [   -np.pi/2,   0,          625,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          110,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [160, 137.5, 150, 270, 120, 270]
        self.jointMin = [-160, -137.5, -150, -270, -105, -270]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper