from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class igus_ReBel_6DOF(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          252,        0           ],
                        [   -np.pi/2,   0,          0,          -np.pi/2    ],
                        [   0,          237,        0,          -np.pi/2    ],
                        [   -np.pi/2,   0,          297,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          126,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [179, 140, 140, 179, 95, 179]
        self.jointMin = [-179, -80, -80, -179, -95, -179]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper