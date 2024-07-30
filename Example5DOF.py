from robotKinematics.src import *
from robotKinematics.inverseKinematics.__5DOF.KUKA_youBot import *
from robotKinematics.inverseKinematics.__5DOF.Motoman_MPL80 import *

yoBot = KUKA_youBot()
#yoBot = Motoman_MPL80()

joint = [np.deg2rad(30),np.deg2rad(30),np.deg2rad(30),None,np.deg2rad(10),np.deg2rad(30)]
#joint = np.array([30, 30, 30, None, 30, 30])

TBW = yoBot.FK(joint)

printMatrix(TBW, 5)

"""yoBot.jointMax = [360] * 6
yoBot.jointMin = [-360] * 6"""

DOF = angleSetConventions.sixDOF(TBW, "ZYX")

printMatrix(DOF, 5)

solution = yoBot.IK(TBW)

printMatrix(solution[:8], 5)