from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Denso_Cobotta(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          180,        0           ],
                        [   -np.pi/2,   0,          20,         -np.pi/2    ],
                        [   0,          165,        0,          -np.pi/2    ],
                        [   -np.pi/2,   12,         177.5,      0           ],
                        [   np.pi/2,    0,          -64.5,      0           ],
                        [   -np.pi/2,   0,          102,        np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [150, 100, 140, 170, 135, 170]
        self.jointMin = [-150, -60, -18, -170, -95, -170]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper