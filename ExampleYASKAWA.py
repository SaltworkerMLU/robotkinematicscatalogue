from robotKinematics.src import *
from robotKinematics.inverseKinematics.YASKAWA import *

#mini = MotoMINI()
#mini = YaskawaHC20DTP()
mini = YaskawaGP7()

#TBW = mini.FK([np.deg2rad(91), np.deg2rad(87), np.deg2rad(-32), np.deg2rad(12), np.deg2rad(10), np.deg2rad(50)]) # ++----
#TBW = mini.FK([np.deg2rad(45), np.deg2rad(-30), np.deg2rad(-25), np.deg2rad(20), np.deg2rad(-20), np.deg2rad(20)])
#TBW = mini.FK([np.deg2rad(45), np.deg2rad(23), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)])
TBW = mini.FK(np.deg2rad( [-15.21, 83.84, 79.73, -8.85, 8.87, -23.15]))
#TBW = mini.FK(np.deg2rad( [30, 30, 30, 30, 0, 0]))
#TBW = mini.FK(np.deg2rad([-15.21, -83.84, 79.73, -8.85, 8.87, -23.15]))

#mini.jointMax = [360] * 6
#mini.jointMin = [-360] * 6

DOF = angleSetConventions.sixDOF(TBW, "ZYX")
printMatrix(DOF)

solution = mini.IK(TBW)

printMatrix(solution[:8], 5)