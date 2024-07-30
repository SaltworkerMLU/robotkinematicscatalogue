from robotKinematics.src import *
from robotKinematics.inverseKinematics.SCARA import *

#SCARA = MotomanMYS1000()
#SCARA = Brooks_PresiceFlex_400_400()
#SCARA = ABB_IRB_910SC_3()
#SCARA = ABB_IRB_920T_6_055()
#SCARA = ABB_IRB_920T_6_065()
#SCARA = Adept_Cobra_s350()
SCARA = Dobot_M1()

#SCARA.FK_symbolic([0, 0, sym.Symbol("d3"), 0], [sym.Symbol("theta1"), sym.Symbol("theta2"), 0, sym.Symbol("theta4")])

#TBW = SCARA.FK(np.deg2rad([160, 22.38, 420, -97.70])) #-120
#TBW = SCARA.FK(np.deg2rad([69, 22.38, 111, -97.70])) #-120
TBW = SCARA.FK(np.deg2rad([0, 0, 0, 0]))
#TBW = SCARA.FK(np.deg2rad([100, 100, 0, 200]))
TBW = SCARA.FK(np.deg2rad([200.000000, -4.000000, 90.000000, 45.000000]))

#TBW = angleSetConventions.transformMatrix([500, 200, 200, 30], "ZYX")

#SCARA.jointMax = [360, 360, 420., 360]
#SCARA.jointMin = [-360, -360, -360, -360]

printMatrix(TBW, 5)

DOF = angleSetConventions.sixDOF(TBW, "XYZ")

printMatrix(DOF, 5)

solutions = SCARA.IK(TBW)

printMatrix(solutions[:8], 5)