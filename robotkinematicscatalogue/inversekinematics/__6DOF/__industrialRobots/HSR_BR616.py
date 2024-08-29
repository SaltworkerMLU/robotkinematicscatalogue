from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class HSR_BR616(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          587.5,      0           ],
                        [   -np.pi/2,   150,        -365,       0           ],
                        [   0,          730,        0,          np.pi       ],
                        [   -np.pi/2,   0,          720,        0           ],
                        [   np.pi/2,    0,          -97,        0           ],
                        [   -np.pi/2,   0,          127,        np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [154, -2, 437, 360, 161, 360]
        self.jointMin = [-154, -178, -103, -360, -161, -360]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper