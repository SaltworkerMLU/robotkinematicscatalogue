from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class Panasonic_TM_1400(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          420,        0           ],
                        [   -np.pi/2,   160,        0,          -np.pi/2    ],
                        [   0,          580,        0,          0           ],
                        [   -np.pi/2,   210,        640,        0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   -np.pi/2,   24,         125,        np.pi       ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [170, 155, 180, 190, 110, 220]
        self.jointMin = [-170, -90, -85, -190, -130, -220]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1, -1, 1, -1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper