from robotKinematics.source import *
from robotKinematics.forwardKinematics import *
from robotKinematics.trajectoryGeneration import *
from robotKinematics.angleSetConventions import *

from robotKinematics.inverseKinematics.__4DOF.deltaRobot3D import *
from robotKinematics.inverseKinematics.__4DOF.palletizingRobot import *
from robotKinematics.inverseKinematics.__4DOF.SCARARobot import *
from robotKinematics.inverseKinematics.__6DOF.sixDOF import *
from robotKinematics.inverseKinematics.__6DOF.collaborativeRobot import *
from robotKinematics.inverseKinematics.__6DOF.industrialRobot import *


# Robot manufacturers tend to have a predefined set of joint directions for each of their products
inv_joint_DEFAULT = [1] * 6
inv_joint_KUKA = [-1, 1, 1, -1, 1, -1]
inv_joint_YASKAWA = [1, 1, -1, -1, -1, -1]

def printMatrix(TBW, digits=5):
    """
    # Prints array or matrix in a readable format
    """

    # Rounding "ridiculous" near-zero number up to float
    TBW = sym.Matrix(TBW)
    for a in sym.preorder_traversal(TBW):
        if isinstance(a, sym.Float):
            TBW = TBW.subs(a, round(a,digits))

    sym.pprint(TBW)