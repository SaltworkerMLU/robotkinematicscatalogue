from robotkinematicscatalogue.inversekinematics.__6DOF.collaborativeRobot import *

class FAIR_Innovation_FR10(collaborativeRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          180,        0           ],
                        [   np.pi/2,    0,          0,          np.pi       ],
                        [   0,          700,        0,          0           ],
                        [   0,          586,        159,        0           ],
                        [   -np.pi/2,   0,          114,        0           ],
                        [   np.pi/2,    0,          106,        np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [175, 85, 160, 85, 175, 175]
        self.jointMin = [-175, -265, -160, -265, -175, -175]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper