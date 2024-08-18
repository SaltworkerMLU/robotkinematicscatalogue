from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class KUKA_KR8_R2100_2_Arc_HW(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          520,        0           ],
                        [   -np.pi/2,   160,        0,          0           ],
                        [   0,          980,        0,          -np.pi/2    ],
                        [   -np.pi/2,   220,        934.921,    0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          80.421,     np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [185, 65, 175, 165, 140, 350]
        self.jointMin = [-185, -185, -138, -165, -140, -350]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, 1, -1, 1, -1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper