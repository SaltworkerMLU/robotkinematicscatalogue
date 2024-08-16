from robotkinematicscatalogue.inversekinematics.__4DOF.palletizingRobot import *

class DobotMagician(palletizingRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          139,        0           ],
                        [   -np.pi/2,   0,          0,          -np.pi/2    ],
                        [   0,          135,        0,          np.pi/2     ],
                        [   0,          147,        0,          0           ],
                        [   -np.pi/2,   61,         6,          np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [90, 85, 95, 90]
        self.jointMin = [-90, 0, -10, -90]
                        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
        
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 1, 1, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper