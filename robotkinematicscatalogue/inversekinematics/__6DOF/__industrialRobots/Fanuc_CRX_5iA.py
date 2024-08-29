from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Fanuc_CRX_5iA(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          185-185,    0           ],
                        [   -np.pi/2,   0,          0,          -np.pi/2    ],
                        [   0,          410,        0,          0           ],
                        [   -np.pi/2,   0,          430,        0           ],
                        [   np.pi/2,    0,          -130,       0           ],
                        [   -np.pi/2,   0,          145,        0           ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [200, 180, 310, 190, 180, 225]
        self.jointMin = [-200, -180, -310, -190, -180, -225]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1, -1, -1, -1, -1]
        
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 1, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper