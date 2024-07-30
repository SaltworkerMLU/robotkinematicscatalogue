from robotKinematics.src import *
from robotKinematics.inverseKinematics.__6DOF.__other.Annin_Robotics_AR2 import *
from robotKinematics.inverseKinematics.__6DOF.__other.Fanuc_CRX_20iAL import *

others = Annin_Robotics_AR2()
#others = Fanuc_CRX_20iAL()

#joint = np.deg2rad([-10, -90, 90, 111, 0, 0])
joint = np.deg2rad([-45, -45, -45, -45, -45, -45])
#joint = np.deg2rad([7.86, -142.30, 44.59, 89.86, 4.67, -72.20])

# NOTE TO SELF: KUKA_KR3_R540 negative values of joint 1, 4, and 6  must be used # [-10, -100, 111, -166, 100, -333]
#joint = np.deg2rad([10, -100, 111, 166, 100, 333]) # [10, -100, 111, 166, 100, 333]
#joint = np.deg2rad( [50, -83.84, 79.73, -8.85, 8.87, -23.15])
#joint = np.deg2rad( [0, -90, 0, 0, 0, 0])
#joint = np.deg2rad( [0, 0, 0, 0, 0, 0])

TBW = others.FK(joint)

printMatrix(TBW, 5)

DOF = angleSetConventions.sixDOF(TBW, "ZYX")


printMatrix(DOF, 5)

solution = others.IK(TBW)

printMatrix(solution[:8], 5)

#angleSetConventions.sixDOF(np.diag([-1, -1, 1, 1]), "ZYX")