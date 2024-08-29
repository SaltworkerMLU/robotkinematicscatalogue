from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Fanuc_P_200_LA_OL_HB(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          0,          0           ],
                        [   -np.pi/2,   0,          378,        -np.pi/2    ],
                        [   0,          1400,       0,          0           ],
                        [   -np.pi/2,   0,          1362.7,     -np.pi/2    ],
                        [np.deg2rad(70),0,          106.42,     0           ],
                        [-np.deg2rad(70),0,          81.94,      np.pi/2     ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [105, 120, 90, 180, 180, 360]
        self.jointMin = [-105, -45, -85, -180, -180, -360]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1, -1, -1, -1, -1]
        
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 1, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper