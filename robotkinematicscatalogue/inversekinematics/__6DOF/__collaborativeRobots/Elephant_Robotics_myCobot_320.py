from robotkinematicscatalogue.inversekinematics.__6DOF.collaborativeRobot import *

class Elephant_Robotics_myCobot_320(collaborativeRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          173.9,      0           ],
                        [   np.pi/2,    0,          0,          np.pi/2     ],
                        [   0,          135,        0,          0           ],
                        [   0,          120,        88.78,      -np.pi/2    ],
                        [   -np.pi/2,   0,          95,         0           ],
                        [   np.pi/2,    0,          65.5,       0           ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [165, 165, 165, 165, 165, 165] # degrees & mm
        self.jointMin = [-165, -165, -165, -165, -165, -165] # degrees & mm

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper