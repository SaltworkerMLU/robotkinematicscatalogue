from robotkinematicscatalogue.inversekinematics.__6DOF.collaborativeRobot import *

class Elephant_Robotics_myCobot_Pro600(collaborativeRobot):
    
    def __init__(self, base=np.eye(4), gripper=np.eye(4)):
                                                
        # Modified Denavit-Hartenberg parameters (DHM)
        DHM = np.array([[   0,          0,          199.34,     0           ],
                        [   np.pi/2,    0,          0,          0           ],
                        [   0,          250,        0,          0           ],
                        [   0,          250,        108,        np.pi       ],
                        [   -np.pi/2,   0,          109.1,      0           ],
                        [   np.pi/2,    0,          75.86,      0           ]])
         
        self.alpha = DHM[:,0]
        self.a = DHM[:,1]
        self.d = DHM[:,2]
        self.theta = DHM[:,3]
                        
        # Joint limits provided in degrees
        self.jointMax = [180, 90, 150, 80, 168, 174] # degrees & mm
        self.jointMin = [-180, -270, -150, -260, -168, -174] # degrees & mm

        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = [1, -1, -1, -1, 1, -1]
                
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # The base and gripper/end-effector of the robot
        self.TB0 = base
        self.T6W = gripper