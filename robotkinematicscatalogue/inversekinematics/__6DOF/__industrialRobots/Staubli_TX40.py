from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Staubli_TX40(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          320-320,    0           ],
                        [   -np.pi/2,   0,          35,         -np.pi/2    ],
                        [   0,          225,        0,          -np.pi/2    ],
                        [   -np.pi/2,   0,          225,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          65,         np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 125, 138, 270, 133.5, 270]
        self.jointMin = [-180, -125, -138, -270, -120, -270]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper