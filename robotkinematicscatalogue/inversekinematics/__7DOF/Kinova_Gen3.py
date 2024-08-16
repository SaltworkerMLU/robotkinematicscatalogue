from robotkinematicscatalogue.inversekinematics.__6DOF.sixDOF import *

class Kinova_Gen3(sixDOF):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          284.8,      0           ],
                        [   -np.pi/2,   0,          -11.75,     0           ],
                        [   np.pi/2,    0,          420.75,     0           ],
                        [   -np.pi/2,   0,          -12.75,     0           ],
                        [   np.pi/2,    -0.35,      314.35,     0           ],
                        [   -np.pi/2,   0,          0,          0           ],
                        [   np.pi/2,    0,          170.35,     0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [360, 128.9, 360, 147.8, 360, 120.3, 360]
        self.jointMin = [-360, -128.9, -360, -147.8, -360, -120.3, -360]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [-1, 1, -1, 1, -1, 1, -1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper