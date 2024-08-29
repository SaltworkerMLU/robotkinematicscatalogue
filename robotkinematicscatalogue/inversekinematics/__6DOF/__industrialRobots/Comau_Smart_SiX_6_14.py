from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Comau_Smart_SiX_6_14(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          450,        0           ],
                        [   -np.pi/2,   150,        0,          -np.pi/2    ],
                        [   0,          590,        0,          -np.pi/2    ],
                        [   -np.pi/2,   130,        647.07,     0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          95,         np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [170, 155, 0, 210, 130, 400]
        self.jointMin = [-170, -85, -170, -210, -130, -400]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, -1, -1, 1, -1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper