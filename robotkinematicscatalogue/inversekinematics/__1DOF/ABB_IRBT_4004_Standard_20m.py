from robotkinematicscatalogue.forwardKinematics import *

class ABB_IRBT_4004_Standard_20m(forwardKinematics):
    
    def __init__(self, base=angleSetConventions.transformMatrix([645.5, 0, 370.5, 0, 90, 0], "ZYX"), 
                 gripper=angleSetConventions.transformMatrix([0, 0, 0, 00, -90, 0], "ZYX")):

        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          0,          0           ]])
        
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                                
        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = [20000.1]
        self.jointMin = [-0.1]
                
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper