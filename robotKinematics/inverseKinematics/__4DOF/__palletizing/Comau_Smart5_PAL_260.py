from robotKinematics.src import *
from robotKinematics.inverseKinematics.__4DOF.palletizingRobot import *

class Comau_Smart5_PAL_260(palletizingRobot):
    
    def __init__(self, gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          750,        0           ],
                        [   -np.pi/2,   400,        0,          -np.pi/2    ],
                        [   0,          1175,       0,          0           ], #try theta_3 = np.pi/2
                        [   0,          1360,       0,          0           ],
                        [   -np.pi/2,   250,        -250,       np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 95, -68, 2700]
        self.jointMin = [-180, -49, -208, -2700]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, -1, -1, 1, 1]

        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 1, 1, 0, 0]

        # Predefined transform matrices used for inverse kinematics
        self.TB0 = np.eye(4)
        self.TB0[2, 3] = self.d[0]
        self.T6W = gripper