from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class ABB_IRB_5500_27(industrialRobot):
    
    def __init__(self, base=angleSetConventions.transformMatrix([0, 0, 0, 90, 0, 0], "ZYX"), 
                 gripper=angleSetConventions.transformMatrix([0, 46.975, 67.087, 90, 35, 0], "ZYX")):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          436,        0           ],
                        [   -np.pi/2,   0,          -5,          -np.pi/2    ],
                        [   0,          1300,       0,          0           ],
                        [   -np.pi/2,   0,          1510.5,     -np.pi/2    ],
                        [np.deg2rad(35),0,          79.169,     0           ],
                        [-np.deg2rad(70),0,          79.169,     np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [104, 150, 70, 719, 719, 720]
        self.jointMin = [-104, -65, -70, -719, -720, -720]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper