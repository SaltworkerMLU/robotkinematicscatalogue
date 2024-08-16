from robotkinematicscatalogue.inversekinematics.__4DOF.SCARARobot import *

class Brooks_PreciseFlex_400_400(SCARARobot):

    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          31,         0           ],
                        [   0,          0,          0,          0           ],
                        [   0,          225,        0,          0           ],
                        [   np.pi,      210,        0,          0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [401.51, 93, 348, 970]
        self.jointMin = [1.51, -93, 12, -970]
        self.translationalJoint = 1 # The joint using translational movement
                                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1, 1, -1]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper