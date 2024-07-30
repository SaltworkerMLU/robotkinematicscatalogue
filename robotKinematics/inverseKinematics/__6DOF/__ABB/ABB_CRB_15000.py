from robotKinematics.src import *
from robotKinematics.inverseKinematics.__6DOF.industrialRobot import *

class ABB_CRB_15000(industrialRobot):
    
    def __init__(self, gripper=np.diag([-1, -1, 1, 1])):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          265,        0           ],
                        [   -np.pi/2,   0,          0,          -np.pi/2    ],
                        [   0,          444,        0,          0           ],
                        [   -np.pi/2,   110,        470,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   80,         101,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                        
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 180, 85, 180, 180, 180]
        self.jointMin = [-180, -180, -225, -180, -180, -180]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = inv_joint_DEFAULT
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # Predefined transform matrices used for inverse kinematics
        self.TB0 = np.eye(4)
        self.TB0[2, 3] = self.d[0]
        self.T6W = gripper
        self.T6W[0, 3] = self.a[5]
        self.T6W[2, 3] = self.d[5]
