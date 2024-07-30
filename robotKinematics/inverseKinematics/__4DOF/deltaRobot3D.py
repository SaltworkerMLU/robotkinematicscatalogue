from robotKinematics.forwardKinematics import *

# https://github.com/tinkersprojects/Delta-Kinematics-Library/tree/master?tab=readme-ov-file

class deltaRobot(forwardKinematics):
    def __init__(self, armLength, rodLength, BassTri, PlatformTri):
        self.armLength = armLength
        self.rodLength = rodLength
        self.BassTri = BassTri
        self.PlatformTri = PlatformTri

    def FK(self, joint):
        tan30 = np.tan(30 * np.pi/180)
        sin30 = np.sin(30 * np.pi/180)
        tan60 = np.tan(60 * np.pi/180)

        t = (self.BassTri - self.PlatformTri) * tan30 / 2

        # Degree to radian conversion
        joint = np.deg2rad(joint)

        # Define points of circles spanning all 3 joints.
        C0 = [0, 0, 0] # [x1, y1, z1]
        C1 = [0, 0, 0] # [x2, y2, z2]
        C2 = [0, 0, 0] # [x3, y3, z3]

        # XYZ-point of circle spanning joint 0
        C0[2] = -self.armLength * np.sin(joint[0])
        C0[1] = -(t + self.armLength * np.cos(joint[0]))

        # XYZ-point of circle spanning joint 1
        C1[2] = -self.armLength * np.sin(joint[1])
        C1[1] = (t + self.armLength * np.cos(joint[1])) * sin30
        C1[0] = C1[1] * tan60

        # XYZ-point of circle spanning joint 2
        C2[2] = -self.armLength * np.sin(joint[2])
        C2[1] = (t + self.armLength * np.cos(joint[2])) * sin30
        C2[0] = -C2[1] * tan60

        # dmn = (y2 - y1) * x3 - (y3 - y1) * x2
        dnm = (C1[1]-C0[1]) * C2[0] - (C2[1]-C0[1]) * C1[0]

        # Get radius squared of each circle: r^2 = x^2 + y^2 + z^2
        w1 = np.dot(C0, C0) # C0[1]**2 + C0[2]**2
        w2 = np.dot(C1, C1) # C1[0]**2 + C1[1]**2 + C1[2]**2
        w3 = np.dot(C2, C2) # C2[0]**2 + C2[1]**2 + C2[2]**2

        a1 = (C1[2] - C0[2]) * (C2[1] - C0[1]) - (C2[2] - C0[2]) * (C1[1] - C0[1])
        b1 = -((w2 - w1) * (C2[1] - C0[1]) - (w3 - w1) * (C1[1] - C0[1])) / 2
        
        a2 = -(C1[2] - C0[2]) * C2[0] + (C2[2] - C0[2]) * C1[0]
        b2 = ((w2 - w1) * C2[0] - (w3 - w1) * C1[0]) / 2

        # Calculate polynomial coefficients
        aV = a1**2 + a2**2 + dnm**2
        bV = 2 * (a1 * b1 + a2 * (b2 - C0[1] * dnm) - C0[2] * dnm**2)
        cV = (b2 - C0[1] * dnm) * (b2 - C0[1] * dnm) + b1**2 + dnm**2 * (C0[2]**2 - self.rodLength**2)

        # Discriminant
        dV = bV**2 - 4 * aV * cV
        
        # Calculate XYZ-coordinates
        z = -0.5 * (bV + np.sqrt(dV)) / aV
        x = (a1 * z + b1) / dnm
        y = (a2 * z + b2) / dnm

        return [x, y, z]
    
    def calcAngleZY(self, XYZ):
        tan30 = np.tan(30 * np.pi/180)

        y1 = -0.5 * tan30 * self.BassTri
        y0 = XYZ[1] - 0.5 * tan30 * self.PlatformTri

        aV = (XYZ[0]**2 + y0**2 + XYZ[2]**2 + self.armLength**2 - self.rodLength**2 - y1**2) / (2 * XYZ[2])
        bV = (y1 - y0) / XYZ[2]

        dV = -(aV + bV * y1) * (aV + bV * y1) + self.armLength * (bV**2 * self.armLength + self.armLength)
        
        yj = (y1 - aV * bV - np.sqrt(dV)) / (bV**2 + 1)
        zj = aV + bV * yj

        return np.arctan2( -zj, (y1 - yj)) * 180 / np.pi
    
    def IK(self, XYZ):
        IKsolution = [0, 0, 0]

        cos120 = np.cos(120 * np.pi/180)
        sin120 = np.sin(120 * np.pi/180)

        IKsolution[0] = self.calcAngleZY(XYZ)
        
        XYZ1 = [XYZ[0] * cos120 + XYZ[1] * sin120, XYZ[1] * cos120 - XYZ[0] * sin120, XYZ[2]]
        IKsolution[1] = self.calcAngleZY(XYZ1)
        
        XYZ2 = [XYZ[0] * cos120 - XYZ[1] * sin120, XYZ[1] * cos120 + XYZ[0] * sin120, XYZ[2]]
        IKsolution[2] = self.calcAngleZY(XYZ2)

        return IKsolution