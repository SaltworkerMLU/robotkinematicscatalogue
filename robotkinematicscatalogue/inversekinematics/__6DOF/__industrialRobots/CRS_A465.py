from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class CRS_A465(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=angleSetConventions.transformMatrix([0, 0, 0, 0, -90, 0], "ZYX")):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          330,        0           ],
                        [   -np.pi/2,   0,          0,          -np.pi/2    ],
                        [   0,          305,        0,          -np.pi/2    ],
                        [   -np.pi/2,   0,          330,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          76,         np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [175, 90, 110, 180, 105, 180]
        self.jointMin = [-175, -90, -110, -180, -105, -180]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper