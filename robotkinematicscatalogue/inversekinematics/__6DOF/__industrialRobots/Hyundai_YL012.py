from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Hyundai_YL012(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          278-278,    0           ],
                        [   -np.pi/2,   0,          384.668,    0           ],
                        [   0,          674.558,    0,          0           ],
                        [   -np.pi/2,   0,          366.355,    -np.pi/2    ],
                        [np.deg2rad(60),0,          197.916,    0           ],
                        [-np.deg2rad(60),0,          165.676,    np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 270, 180, 180, 180, 180]
        self.jointMin = [-180, -90, -180, -180, -180, -180]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, -1, -1, 1, 1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper