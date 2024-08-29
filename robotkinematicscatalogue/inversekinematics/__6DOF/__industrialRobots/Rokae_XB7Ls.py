from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Rokae_XB7Ls(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          379,        0           ],
                        [   -np.pi/2,   30,         0,          -np.pi/2    ],
                        [   0,          440,        0,          0           ],
                        [   -np.pi/2,   35,         433.5,      0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          82.1,       np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [165, 130, 65, 165, 130, 355]
        self.jointMin = [-165, -90, -205, -165, -130, -355]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper