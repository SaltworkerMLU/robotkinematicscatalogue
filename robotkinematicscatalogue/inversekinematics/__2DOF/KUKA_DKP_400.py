from robotkinematicscatalogue.forwardKinematics import *

class KUKA_DKP_400(forwardKinematics):
    
    def __init__(self, base=angleSetConventions.transformMatrix([0, 0, 510, 90, 0, 90], "ZYX"), gripper=np.eye(4)):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          0,          0           ],
                        [   -np.pi/2,   0,          347,        -np.pi/2    ]])
        
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [90, 190]
        self.jointMin = [-90, -190]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, 1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper