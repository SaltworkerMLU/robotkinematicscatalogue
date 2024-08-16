from robotkinematicscatalogue.inversekinematics.__6DOF.collaborativeRobot import *

class Dobot_CR3(collaborativeRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          134.8,      0           ],
                        [   np.pi/2,    0,          0,          np.pi/2     ],
                        [   0,          274,        0,          0           ],
                        [   0,          230,        128.3,      -np.pi/2    ],
                        [   -np.pi/2,   0,          116,        0           ],
                        [   np.pi/2,    0,          105,        0           ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [360, 360, 155, 360, 360, 360] # degrees & mm
        self.jointMin = [-360, -360, -155, -360, -360, -360] # degrees & mm

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper