from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class QJAR_QJR20_1600(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          537,        0           ],
                        [   -np.pi/2,   149,        0,          -np.pi/2    ],
                        [   0,          730,        0,          0           ],
                        [   -np.pi/2,   120,        780,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          111,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [172, 147, 83, 167, 115, 355]
        self.jointMin = [-172, -115, -98, -167, -115, -355]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper