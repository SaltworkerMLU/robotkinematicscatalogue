from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class uFactory_xArm5(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          267,        0           ],
                        [   -np.pi/2,   0,          0,          np.deg2rad(-79.35) ],
                        [   0,          289.489,    0,          np.deg2rad(79.35) ],
                        [   -np.pi/2,   77.5,       342.5,      0           ],
                        [   np.pi/2,    0,          0,          0           ], # theta5 = np.pi/2
                        [   -np.pi/2,   76,         97,         0           ]])

        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                                        
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [360, 120, 11, 0.0001, 180, 360]
        self.jointMin = [-360, -118, -225, -0.0001, -97, -360]
        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper