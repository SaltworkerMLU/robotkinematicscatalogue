from robotKinematics.src import *
from robotKinematics.inverseKinematics.UniversalRobots import *
from robotKinematics.inverseKinematics.__6DOF.__other.AUBO_i3 import *
from robotKinematics.inverseKinematics.__6DOF.__other.AUCTECH_X3_618 import *
from robotKinematics.inverseKinematics.__6DOF.__other.TM5_700 import *

Uni = UR3()
#Uni = AUBO_i3()
#Uni = AUCTECH_X3_618()
#Uni = TM5_700()

joint = np.deg2rad([90,90,60,-30,20,-120])
#joint = np.deg2rad([25, 0.001, 0.001, 0.001, 0.001, 0.001])
#joint = np.deg2rad([45, 45, 45, 45, 45, 45])
#joint = np.deg2rad([0, -90, -90, 0, 90, 0])
#joint = np.deg2rad([0, 0, 90, 0, 90, 0])
#joint = np.deg2rad([0, 0, -90, 90, 90, 0])
#joint = np.deg2rad([0, 0, 90, -90, 90, 0])

"""TBWS = Uni.FK_symbolic([0] * 6, 
                       [0, 
                        0, 
                        0,
                        sym.Symbol("theta6"),
                        0,
                        0], 4, 4)"""

TBWS = Uni.FK_symbolic([0] * 6, 
                       [0, 
                        0, 
                        0,
                        0,
                        sym.Symbol("theta5"),
                        sym.Symbol("theta6")], 1, 6)

printMatrix(TBWS)

TBW = Uni.FK(joint)

printMatrix(TBW, 5)

DOF = angleSetConventions.sixDOF(TBW, "ZYX")

printMatrix(DOF, 5)

solution = Uni.IK(TBW)

printMatrix(solution[:8], 5)