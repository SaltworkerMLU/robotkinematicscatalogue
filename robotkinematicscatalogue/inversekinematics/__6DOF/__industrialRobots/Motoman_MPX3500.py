from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Motoman_MPX3500(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          800-800,    0           ],
                        [   -np.pi/2,   0,          0,          -np.pi/2    ],
                        [   0,          1300,       0,          0           ],
                        [   -np.pi/2,   0,          1400,       -np.pi/2    ],
                        [np.deg2rad(60),0,          114,        0           ],
                        [-np.deg2rad(60),0,          123,        np.pi/2     ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [150, 140, 90, 720, 720, 720]
        self.jointMin = [-150, -65, -65, -720, -720, -720]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, -1, -1, 1, 1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper