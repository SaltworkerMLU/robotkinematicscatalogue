from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Foxbot_A1700(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          525,        0           ],
                        [   -np.pi/2,   150,        0,          -np.pi/2    ],
                        [   0,          760,        0,          -np.pi/2    ],
                        [   -np.pi/2,   140,        795,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          100,        np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [160, 120, 170, 180, 125, 359]
        self.jointMin = [-160, -50, -30, -180, -125, -359]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper