from robotkinematicscatalogue.inversekinematics.__6DOF.industrialRobot import *

class HSR_BR609(industrialRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          420.5,      0           ],
                        [   -np.pi/2,   0,          -222.5,     0           ],
                        [   0,          626,        0,          np.pi       ],
                        [   -np.pi/2,   0,          530.5,      0           ],
                        [   np.pi/2,    0,          -91,        0           ],
                        [   -np.pi/2,   0,          122,        np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [155, -2, 398.5, 90, 96.5, 360]
        self.jointMin = [-155, -178, -141.5, -90, -96.5, -360]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper