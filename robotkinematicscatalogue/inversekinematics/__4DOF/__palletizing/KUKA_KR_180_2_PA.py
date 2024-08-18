from robotkinematicscatalogue.inversekinematics.__4DOF.palletizingRobot import *

class KUKA_KR_180_2_PA(palletizingRobot):

    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          670,        0           ],
                        [   -np.pi/2,   350,        0,          0           ],
                        [   0,          1250,       0,          0           ],
                        [   0,          1350,       0,          0           ],
                        [   -np.pi/2,   250,        180,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [185, 0, 161, 350]
        self.jointMin = [-185, -140, -19, -350]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, 1, 1, -1, 1]

        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 2, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper