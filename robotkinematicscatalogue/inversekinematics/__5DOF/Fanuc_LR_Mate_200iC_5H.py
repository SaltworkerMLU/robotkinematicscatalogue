from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Fanuc_LR_Mate_200iC_5H(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          330-330,    0           ],
                        [   -np.pi/2,   75,         0,          -np.pi/2    ],
                        [   0,          300,        0,          0           ],
                        [   -np.pi/2,   75,         320,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          80,         0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [170, 140, 170, 190, 115, 0.0001]
        self.jointMin = [-170, -60, -70, -190, -115, -0.0001]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
        
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper