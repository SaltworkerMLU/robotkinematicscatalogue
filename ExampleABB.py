from robotKinematics.src import *
from robotKinematics.inverseKinematics.ABB import *

#ABB140 = ABB_IRB_140()
ABB140 = ABB_IRB_1300_11()
#ABB140 = ABB_CRB_15000()

#joint = np.deg2rad([30, 30, 30, 30, 30, 30])
#joint = np.deg2rad([30, 30, 40, -10, 10, -10])
joint = np.deg2rad( [-45, -83.84, 29.73, -8.85, 8.87, -23.15])
#joint = np.deg2rad( [0, 42.5, 32.5, 0, 0.1, 0])
#joint = np.deg2rad([-45, -45, -45, -45, -45, -45])

#ABB140.jointMax = [360] * 6
#ABB140.jointMin = [-360] * 6

TBW = ABB140.FK(joint)
TBWS = ABB140.FK_symbolic([0, 0, 0, 0, 0, 0],
                   [0, 0, 0, sym.Symbol("theta4"), sym.Symbol("theta5"), sym.Symbol("theta6")], 4, 6 )

printMatrix(TBWS)

printMatrix(TBW, 5)

DOF = angleSetConventions.sixDOF(TBW, "ZYX")

printMatrix(DOF, 5)


solution = ABB140.IK(TBW)

printMatrix(solution[:8], 5)