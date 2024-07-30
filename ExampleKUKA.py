from robotKinematics.src import *
from robotKinematics.inverseKinematics.KUKA import *
from robotKinematics.inverseKinematics.__6DOF.__other.Automata_EVA import *

#KUKA_es = KUKA_iisy_3_R930()
KUKA_es = KUKA_KR3_R540()
#KUKA_es = Automata_EVA()

#joint = np.deg2rad([-10, -90, 90, 111, 0, 0])
#joint = np.deg2rad([-45, -45, -45, -45, -45, -45])
#joint = np.deg2rad([7.86, -142.30, 44.59, 89.86, 4.67, -72.20])

# NOTE TO SELF: KUKA_KR3_R540 negative values of joint 1, 4, and 6  must be used # [-10, -100, 111, -166, 100, -333]
#joint = np.deg2rad([10, -100, 111, 166, 100, 333]) # [10, -100, 111, 166, 100, 333]
joint = np.deg2rad( [-15.21, -83.84, 79.73, -8.85, 8.87, -23.15]) #[-15.21, -83.84+90, 79.73-90, -8.85, 8.87, -23.15]

#KUKA_es.jointMax = [360] * 6
#KUKA_es.jointMin = [-360] * 6
#joint = np.deg2rad( [0, 0, 0, 0, 0, 0])
#joint = np.deg2rad( [-45, -45, -45, -45, -45, -45] )

TBWS = KUKA_es.FK_symbolic([0] * 6, [0, 0, 0, 
                                    sym.Symbol("theta4"), 
                                    sym.Symbol("theta5"),
                                    sym.Symbol("theta6"), 4, 6])

printMatrix(TBWS)

TBW = KUKA_es.FK(joint)

printMatrix(TBW, 5)

DOF = angleSetConventions.sixDOF(TBW, "ZYX")

printMatrix(DOF, 5)

solution = KUKA_es.IK(TBW)

printMatrix(solution, 5)

#angleSetConventions.sixDOF(np.diag([-1, -1, 1, 1]), "ZYX")