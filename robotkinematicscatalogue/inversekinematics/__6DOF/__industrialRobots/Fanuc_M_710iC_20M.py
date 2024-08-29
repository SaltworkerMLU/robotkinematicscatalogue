from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Fanuc_M_710iC_20M(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          565-565,    0           ],
                        [   -np.pi/2,   150,        0,          -np.pi/2    ],
                        [   0,          870,        0,          0           ],
                        [   -np.pi/2,   190,        1550,       0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          100,        0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 135, 275, 200, 140, 450]
        self.jointMin = [-180, -90, -90, -200, -140, -450]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1, -1, -1, -1, -1]
        
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 1, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper