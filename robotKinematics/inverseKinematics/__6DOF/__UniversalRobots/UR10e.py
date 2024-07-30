from robotKinematics.src import *
from robotKinematics.inverseKinematics.__6DOF.collaborativeRobot import *

class UR10e(collaborativeRobot):
    
    def __init__(self, gripper=np.diag([-1, -1, 1, 1])):
                                                        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          180.7,      0           ],
                        [   np.pi/2,    0,          0,          np.pi       ],
                        [   0,          612.7,      0,          0           ],
                        [   0,          571.335,    174.15,     0           ],
                        [   -np.pi/2,   0,          119.85,     0           ],
                        [   np.pi/2,    0,          116.55,     np.pi       ]])
        
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [360, 360, 360, 360, 360, 360]
        self.jointMin = [-360, -360, -360, -360, -360, -360]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = inv_joint_DEFAULT
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # Predefined transform matrices used for inverse kinematics
        self.TB0 = np.eye(4)
        self.TB0[2, 3] = self.d[0]
        self.T6W = gripper