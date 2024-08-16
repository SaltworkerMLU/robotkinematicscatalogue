from robotkinematicscatalogue.inversekinematics.__6DOF.collaborativeRobot import *

class FAIR_Innovation_FR3(collaborativeRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          140,        0           ],
                        [   np.pi/2,    0,          0,          np.pi       ],
                        [   0,          280,        0,          0           ],
                        [   0,          240,        102,        0           ],
                        [   -np.pi/2,   0,          102,        0           ],
                        [   np.pi/2,    0,          100,        np.pi       ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [175, 85, 150, 85, 175, 175]
        self.jointMin = [-175, -265, -150, -265, -175, -175]

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper