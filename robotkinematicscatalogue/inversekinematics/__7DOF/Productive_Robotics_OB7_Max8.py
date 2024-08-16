from robotkinematicscatalogue.inversekinematics.__6DOF.sixDOF import *

class Productive_Robotics_OB7_Max8(sixDOF):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          495.678,    0           ],
                        [   -np.pi/2,   0,          293.941,    0           ],
                        [   np.pi/2,    0,          867.8,      0           ],
                        [   -np.pi/2,   0,          -209,       0           ],
                        [   np.pi/2,    0,          665.74,     0           ],
                        [   -np.pi/2,   0,          183.94,     0           ],
                        [   np.pi/2,    0,          166.24,     0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [160, 130, 170, 154, 170, 260, 170]
        self.jointMin = [-160, -130, -170, -107, -170, -80, -170]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 7
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper