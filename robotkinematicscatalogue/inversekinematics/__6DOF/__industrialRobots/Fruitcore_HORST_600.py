from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Fruitcore_HORST_600(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          340,        0           ],
                        [   -np.pi/2,   -33.5,      0,          -np.pi/2    ],
                        [   0,          300,        0,          0           ],
                        [   -np.pi/2,   0,          260,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          62,         np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [173, 115, 41, 172, 142, 300]
        self.jointMin = [-173, -64, -176, -172, -142, -300]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper