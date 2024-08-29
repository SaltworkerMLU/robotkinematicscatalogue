from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Kawasaki_BX100N(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          465-465,    np.pi/2     ],
                        [   -np.pi/2,   200,        0,          -np.pi/2    ],
                        [   0,          880,        0,          0           ],
                        [   -np.pi/2,   210,        1100,       0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          225,        np.pi/2     ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [160, 120, 90, 210, 125, 210]
        self.jointMin = [-160, -60, -77, -210, -125, -210]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, -1, 1, -1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper