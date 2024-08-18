from robotkinematicscatalogue.forwardKinematics import *

class ABB_IRBP_A250_D1000(forwardKinematics):
    
    def __init__(self, base=angleSetConventions.transformMatrix([0, 0, 1020, 90, 0, -90], "ZYX"), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          0,          0           ],
                        [   np.pi/2,    0,          -120,       np.pi/2     ]])
        
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [180, 720]
        self.jointMin = [-180, -720]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper