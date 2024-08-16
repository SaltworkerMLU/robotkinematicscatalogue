from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class KUKA_KR1000_L950_titan_PA(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          1100,       0           ],
                        [   -np.pi/2,   600,        0,          0           ],
                        [   0,          1400,       0,          -np.pi/2    ],
                        [   -np.pi/2,   65,         1600,       0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          372,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                        
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 140, 40, 0.0001, 145, 360]
        self.jointMin = [-180, -105, -205, -0.0001, -145, -360]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, 1, -1, 1, -1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper