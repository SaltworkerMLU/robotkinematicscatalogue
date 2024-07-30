from robotKinematics.src import *

GitDelta = deltaRobot(70, 300, 139, 112)
#GitDelta = deltaRobot(440.483, 874.686, 370, 96/2) # ABB IRB 360 - sumn
#GitDelta = deltaRobot(-432, 1244, 164, 44) # ABB IRB 360 - 1600

# https://people.ohio.edu/williams/html/PDF/DeltaKin.pdf

resy = GitDelta.FK([-8.275,-8.275,-8.275]) 
#[23.2, 60.9, 10.2]


result = GitDelta.IK(resy)

print(result)

#print(GitDelta.IKsol)

print(resy)