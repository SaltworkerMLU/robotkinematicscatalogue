from robotKinematics.src import *
from robotKinematics.inverseKinematics.__4DOF.palletizingRobot import *

class KUKA_KR_40_PA(palletizingRobot):

    def __init__(self, gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          620,        0           ],
                        [   -np.pi/2,   220,        0,          0           ],
                        [   0,          850,        0,          0           ],
                        [   0,          850,        0,          0           ],
                        [   -np.pi/2,   200,        190,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [155, -15, 145, 350]
        self.jointMin = [-155, -120, 15, -350]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = inv_joint_KUKA

        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 2, 0, 0]

        # Predefined transform matrices used for inverse kinematics
        self.TB0 = np.eye(4)
        self.TB0[2, 3] = self.d[0]
        self.T6W = gripper