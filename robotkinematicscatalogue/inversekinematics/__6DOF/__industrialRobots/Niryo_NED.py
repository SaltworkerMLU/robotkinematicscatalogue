from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Niryo_NED(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          169.5,      0           ],
                        [   -np.pi/2,   0,          0,          -np.deg2rad(86.887)],
                        [   0,          221.325,    0,          -np.deg2rad(3.112)],
                        [   -np.pi/2,   32.406,     235.556,    0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   9.25,       41.2,       np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [170, 35, 90, 120, 55, 145]
        self.jointMin = [-170, -120, -77, -120, -100, -145]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, -1, -1, 1, -1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper