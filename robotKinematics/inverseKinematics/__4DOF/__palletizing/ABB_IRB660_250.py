from robotKinematics.src import *
from robotKinematics.inverseKinematics.__4DOF.palletizingRobot import *

class ABB_IRB660_250(palletizingRobot):
    
    def __init__(self, gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          800,        0           ],
                        [   -np.pi/2,   300,        0,          -np.pi/2    ],
                        [   0,          1280,       0,          np.pi/2     ],
                        [   0,          1350,       0,          0           ],
                        [   -np.pi/2,   260,        247,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 82, 120, 300]
        self.jointMin = [-180, -42, -20, -300]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = inv_joint_DEFAULT

        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 1, 1, 0, 0]

        # Predefined transform matrices used for inverse kinematics
        self.TB0 = np.eye(4)
        self.TB0[2, 3] = self.d[0]
        self.T6W = gripper