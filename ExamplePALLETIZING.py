from robotKinematics.src import *
from robotKinematics.inverseKinematics.palletizing import *
#from robotKinematics.inverseKinematics.__4DOF.__palletizing.KUKA_KR_700_PA import *

#pall = ABB_IRB260()
#pall = ABB_IRB460()
#pall = ABB_IRB660_180()
#pall = ABB_IRB660_250()
#pall = ABB_IRB760()
#pall = Comau_Smart5_PAL_180()
#pall = Comau_Smart5_PAL_260()
#pall = DobotMagician()
#pall = DobotMagicianLite()
#pall = DobotMG400()
#pall = Fanuc_M410iB_300()
#pall = Fanuc_M410iB_700()
#pall = Fanuc_M410iC_185()
#pall = GSK_RMD08()
#pall = KUKA_KR_40_PA()
#pall = KUKA_KR_100_2_PA()
#pall = KUKA_KR_120_R3200_PA()
#pall = KUKA_KR_140_R3200_2_PA()
#pall = KUKA_KR_180_R3200_PA()
#pall = KUKA_KR_180_R3200_2_PA()
#pall = KUKA_KR_180_2_PA()
#pall = KUKA_KR_240_PA()
#pall = KUKA_KR_240_R3200_PA()
#pall = KUKA_KR_700_PA()
#pall = Leantec_LP1520_12_C()
#pall = RoboDK_RDK_2100_PA()
#pall = uFactory_uArm()
#pall = Motoman_EPL160()
#pall = Motoman_MPK50()
#pall = Motoman_MPL300()
#pall = Motoman_MPL800()
#pall = Motoman_MPL800II()
pall = Motoman_PL190()

joint = np.deg2rad([20, 40, -60, -111])
#joint = np.deg2rad([0, 0, 0, 0])
#joint = np.deg2rad([0, 20, -20, 30])
#joint = np.deg2rad([20, 30, -90, 33])
#joint = np.deg2rad([0, -90, 90, 0])
#joint = np.deg2rad([20, -40, 119, 179])
#joint = [np.deg2rad(20), np.deg2rad(1), np.deg2rad(-1), np.deg2rad(-179)]
#joint = [np.deg2rad(0), np.deg2rad(-90), np.deg2rad(90), np.deg2rad(0)]
#joint = [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]

"""TBWS = pall.FK_symbolic([0, 0, 0, 0, 0], [sym.Symbol("theta1"), sym.Symbol("theta2"), 
                                    sym.Symbol("theta3"), 0, sym.Symbol("theta4")])

printMatrix(TBWS)"""

pall.jointMax = [360, 360, 360, 360]
pall.jointMin = [-360, -360, -360, -360]

TBW = pall.FK(joint)

printMatrix(TBW)

#angleSetConventions.sixDOF(TBW, "xyz")
DOF = angleSetConventions.sixDOF(TBW, "ZYX")

printMatrix(DOF)

solution = pall.IK(TBW)

printMatrix(solution)