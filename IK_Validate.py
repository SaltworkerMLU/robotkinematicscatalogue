from robotkinematicscatalogue.synthesize import *

#usedRobot = Adept_Cobra_s800()
#usedRobot = ABB_IRB260()
#usedRobot = KUKA_KR3_R540()
usedRobot = UR5()

# Use random joint values spanning joint limits of used robot
joint = np.empty(len(usedRobot.jointMin))
for i in range(len(joint)):
    joint[i] = np.random.uniform(usedRobot.jointMin[i], usedRobot.jointMax[i])

printMatrix(joint)

# SLIGHT ISSUE: Also converts translational joint(s) to "radians"
TBW = usedRobot.FK(np.deg2rad(joint))

print("Forward Kinematics:\n")
printMatrix(TBW)

print("\n 6DOF ZYX euler angle representation:")
printMatrix(angleSetConventions.sixDOF(TBW, "ZYX"))

solutions, wrongSolutions = usedRobot.IK(TBW)
printMatrix(solutions[:8,:])

# Print test results
message = 'IK VALIDATION RESULT:\n' + ' -> '+ str(len(solutions)) + ' total IK solutions\n' + ' -> '+ str(len(wrongSolutions)) + ' incorrect IK solutions'

print(message)