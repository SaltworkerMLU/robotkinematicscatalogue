from robotkinematicscatalogue.inversekinematics.__6DOF.collaborativeRobot import *

class Hans_S20_1700(collaborativeRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                        
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          171,        0           ],
                        [   np.pi/2,    0,          0,          np.pi       ],
                        [   0,          850,        0,          0           ],
                        [   0,          712,        179,        0           ],
                        [   -np.pi/2,   0,          138,        0           ],
                        [   np.pi/2,    0,          125.7,      np.pi       ]])
 
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [360, 10, 168, 360, 360, 360]
        self.jointMin = [-360, -190, -168, -360, -360, -360]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1] * 6
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper