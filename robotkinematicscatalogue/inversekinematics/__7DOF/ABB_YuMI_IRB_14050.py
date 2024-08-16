from robotkinematicscatalogue.inversekinematics.__6DOF.sixDOF import *

class ABB_YuMI_IRB_14050(sixDOF):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          305,        0           ],
                        [   -np.pi/2,   -30,        0,          0           ],
                        [   np.pi/2,    30,         251.5,      0           ],
                        [   -np.pi/2,   40.5,       0,          np.pi/2     ],
                        [   np.pi/2,    -40.5,      265,        0           ],
                        [   -np.pi/2,   27,         0,          0           ],
                        [   np.pi/2,    -27,        36,         0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [168.5, 143.5, 290, 80, 138, 229, 168.5]
        self.jointMin = [-168.5, -143.5, -290, -123.5, -88, -229, -168.5]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 7
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper