from robotkinematicscatalogue.inversekinematics.__4DOF.palletizingRobot import *

class RoboDK_RDK_2100_PA(palletizingRobot):

    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          630,        0           ],
                        [   -np.pi/2,   150,        0,          -np.pi/2    ],
                        [   0,          880,        0,          np.pi/2     ],
                        [   0,          860,        0,          0           ],
                        [   -np.pi/2,   225,        140,        np.pi       ]])
        
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 120, 140, 300]
        self.jointMin = [-180, -60, -90, -300]
                        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
        
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 1, 1, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper