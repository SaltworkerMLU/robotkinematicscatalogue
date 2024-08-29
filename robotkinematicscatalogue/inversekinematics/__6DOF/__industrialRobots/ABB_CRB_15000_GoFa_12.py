from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class ABB_CRB_15000_GoFa_12(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          338,        0           ],
                        [   -np.pi/2,   0,          0,          -np.pi/2    ],
                        [   0,          707,        0,          0           ],
                        [   -np.pi/2,   110,        534,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   80,         101,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                        
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [270, 180, 85, 180, 180, 270]
        self.jointMin = [-270, -180, -225, -180, -180, -270]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper