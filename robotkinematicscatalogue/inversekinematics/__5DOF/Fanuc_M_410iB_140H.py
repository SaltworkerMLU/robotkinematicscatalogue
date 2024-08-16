from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Fanuc_M_410iB_140H(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          720-720,    0           ],
                        [   -np.pi/2,   242,        0,          -np.pi/2    ],
                        [   0,          1160,       0,          0           ],
                        [   -np.pi/2,   150,        1730,       0           ],
                        [   np.pi/2,    0,          0,          np.pi/2     ],
                        [   -np.pi/2,   0,          215,        0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 110, 355, 0.0001, 220, 360]
        self.jointMin = [-180, -110, -220, -0.0001, -40, -360]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1, -1, -1, -1, -1]
        
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 1, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper