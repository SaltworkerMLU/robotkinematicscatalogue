from robotKinematics.source import *
from robotKinematics.angleSetConventions import *

class forwardKinematics:
    """
    # This class does symbolic forward kinematics.
    In other words: the used Denavit-Hartemberg (Modified) parameters are symbolic.
    """

    def __init__(self, DHM : np.matrix, gripper : np.matrix, jointMax : list = [1] * 6, jointMin : list = [1] * 6, inv_joint : list = [1] * 6) -> None:
        """
        The class constructor initializes the parameters as class attributes.

        Parameters
        ---
        DHM : np.matrix
            The Denavit-Hartenberg Modified parameters used.
        
        gripper : np.matrix
            The transform matrix of the used gripper. Presumed equal to the identity matrix.
        """

        DHM = np.array(DHM)

        # Modified Denavit-Hartenberg parameters (DHM)
        self.alpha = DHM[:,0] # radians
        self.a = DHM[:,1] # mm
        self.d = DHM[:,2] # mm
        self.theta = DHM[:,3] # radians

        # Joint limits provided in degrees and/or millimeters (mm)
        self.jointMax = jointMax
        self.jointMin = jointMin
                        
        # Identify inverted joints - joints that rotate counterclockwise in local z-axis frame
        self.inv_joint = inv_joint
        
        # Identify nullified joints - prior joints that cancel out at specified self.null_joint element
        self.null_joint = [0, 0, 0, 0, 0, 0]

        # Predefined transform matrices used for inverse kinematics
        self.TB0 = np.eye(4)
        self.TB0[2, 3] = self.d[0]
        self.T6W = gripper

    def FK_symbolic(self, transl_joint : list, rot_joint : list, start : int = 1, ender : int = 0) -> np.matrix:
        """
        Do symbolic forward kinematics. 

        Parameters
        ---

        transl_joint : sym.Symbol(float[6])
            Translational value of each joint

        rot_joint : sym.Symbol(float[6])
            Rotational value of each joint, regardless of the values of @start and @ender

        start : int
            The start joint to do symbolic forward kinematics on.

        ender : int
            The end joint to do symbolic forward kinematics on.
        """

        # By default, set end joint equal last joint
        if ender == 0:
            ender = len(self.alpha)

        IOtheta = 6 * [None] # = [None, None, None, None, None, None]
        IOd = 6 * [None]
              
        for i in range(start-1, ender):
            IOd[i] = self.d[i] + self.inv_joint[i] * transl_joint[i]

        for i in range(start-1, ender):
            IOtheta[i] = self.theta[i] + self.inv_joint[i] * rot_joint[i]

        TBW = sym.Matrix(sym.eye(4))
        for i in range(start-1, ender):
            temp = sym.Matrix([
                [sym.cos(IOtheta[i]), -sym.sin(IOtheta[i]), 0, self.a[i]],
                [sym.sin(IOtheta[i]) * sym.cos(self.alpha[i]), sym.cos(IOtheta[i]) * sym.cos(self.alpha[i]), -sym.sin(self.alpha[i]), -sym.sin(self.alpha[i]) * IOd[i]],
                [sym.sin(IOtheta[i]) * sym.sin(self.alpha[i]), sym.cos(IOtheta[i]) * sym.sin(self.alpha[i]), sym.cos(self.alpha[i]), sym.cos(self.alpha[i]) * IOd[i]],
                [0, 0, 0, 1]
            ])
            TBW = TBW.multiply(temp)

        return TBW