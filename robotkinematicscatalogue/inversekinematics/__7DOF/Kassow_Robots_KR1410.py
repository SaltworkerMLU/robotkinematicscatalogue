from robotkinematicscatalogue.inversekinematics.__6DOF.sixDOF import *

class Kassow_Robots_KR1410(sixDOF):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          357.2,      0           ],
                        [   -np.pi/2,   62,         105.2,      0           ],
                        [   np.pi/2,    -62,        722.5,      0           ],
                        [   -np.pi/2,   50,         -85.7,      0           ],
                        [   np.pi/2,    -50,        677.5,      0           ],
                        [   -np.pi/2,   0,          111,        0           ],
                        [   np.pi/2,    0,          108,        0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [360, 180, 360, 180, 360, 360, 360]
        self.jointMin = [-360, -70, -360, -70, -360, -360, -360]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 7
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper