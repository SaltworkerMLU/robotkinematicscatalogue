from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Comau_N_220_27(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          775,        0           ],
                        [   -np.pi/2,   400,        0,          -np.pi/2    ],
                        [   0,          1050,       0,          -np.pi/2    ],
                        [   -np.pi/2,   280,        1220,       0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          229.5,      np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [180, 76, 74, 200, 125, 400]
        self.jointMin = [-180, -77, -167, -200, -125, -400]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, -1, -1, 1, -1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper