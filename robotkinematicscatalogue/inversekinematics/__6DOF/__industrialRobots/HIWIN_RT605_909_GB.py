from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class HIWIN_RT605_909_GB(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          375-375,    0           ],
                        [   -np.pi/2,   30,         3,          -np.pi/2    ],
                        [   0,          440,        0,          0           ],
                        [   -np.pi/2,   40,         438,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          86.5,       np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [165, 85, 185, 190, 115, 360]
        self.jointMin = [-165, -125, -55, -190, -115, -360]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, -1, -1, 1, -1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper