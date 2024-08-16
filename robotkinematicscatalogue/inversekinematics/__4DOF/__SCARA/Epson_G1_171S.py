from robotkinematicscatalogue.inversekinematics.__4DOF.SCARARobot import *

class Epson_G1_171S(SCARARobot):

    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          0,          0           ],
                        [   0,          75,         0,          0           ],
                        [   0,          100,        0,          0           ],
                        [   np.pi,      0,          0,          0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [215, 140, 0.1, 360]
        self.jointMin = [-35, -140, -100.1, -360]
        self.translationalJoint = 3 # The joint using translational movement
                                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1, 1, -1]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper