from robotkinematicscatalogue.inversekinematics.__6DOF.sixDOF import *

class Agile_Robots_Diana_7(sixDOF):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   np.pi,      0,          -285.6,     0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   0,          -458.6,     0           ],
                        [   np.pi/2,    65,         0,          0           ],
                        [   -np.pi/2,   -52.8,      -455.4,     0           ],
                        [   -np.pi/2,   -12.2,      0,          np.pi       ],
                        [   -np.pi/2,   87,         -116.9,     0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [179, 90, 179, 175, 179, 179, 179]
        self.jointMin = [-179, -90, -179, 0, -179, -179, -179]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 7
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper