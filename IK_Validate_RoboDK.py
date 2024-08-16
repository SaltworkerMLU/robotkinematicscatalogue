from robotkinematicscatalogue.synthesize import *
from robodk.robolink import *

"""
Remember to have RoboDK api installed using command prompt. 
If not done, use the command:
$ pip install robodk

PROCEDURE:
1. Define object "usedRobot" as any robot defined in this repository, e.g. 
    - Adept_Cobra_s800()
    - ABB_IRB260()
    - KUKA_KR3_R540()
    - UR5()
2. Open RoboDK and the robot to use for testing (also ensure RoboDK API is set up correctly)
3. Run the program
"""
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

# Connect to the RoboDK API
RDK = Robolink()

# Select robot in use in RoboDK
robot = RDK.ItemUserPick('Select one robot', ITEM_TYPE_ROBOT)
ref = robot.Parent()

# Test each IK solution using roboDK
for i in range(len(solutions)):
    robot.setJoints(solutions[i].tolist())
    target = RDK.AddTarget("Target" + str(i), ref, robot)
    target.setJoints(solutions[i].tolist())

RDK.ShowMessage(message)