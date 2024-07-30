from robotKinematics.source import *

class angleSetConventions:
    """ 
    # This class takes care of any conversions between:
    - Transform matrices
    - Six degrees of freedom (6DOF) positions
    - Quaternions
    - Axis angle sets

    See also
    ---
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_euler.html#from-euler

    """

    def sixDOF(TBW: float, angleSet: str) -> list:
        """
        Converts any transform matrix into a 6DOF position.

        Parameters
        ---
        
        TBW : float[4][4]
            The transform matrix to be converted.

        angleSet : str
            Any angle set convention consists of 3 euler angles defined as a rotation of either x, y, or z.
            Intrinsic rotations use uppercase letters while extrinsic rotation use lower case letters, 
            e.g. "XYZ" uses intrinsic euler angles while "xyz" uses extrinsic euler angles.
        """

        r = R.from_matrix(TBW[:3, :3]).as_euler(angleSet,degrees=True)
        DOF = [ TBW[0, 3], TBW[1, 3], TBW[2, 3], r[0], r[1], r[2] ]

        return DOF # Return orientation

    def sixDOF_quaternions(TBW: np.matrix) -> list:
        """
        Converts any transform matrix into quaternions.

        Parameters
        ---

        TBW : float[4][4]
            The transform matrix in use.
        """

        r = R.from_matrix(TBW[:3, :3]).as_quat()

        return r #[ r[3], r[0], r[1], r[2] ] #Shift elements by one
    
    def transformMatrix(DOF: float, angleSet: str) -> np.matrix:
        """
        Converts any 6DOF position into a transform matrix.

        Parameters
        ---

        DOF : double[6]
            The six degrees of freedom in use.

        angleSet : str
            The angle set convention which @DOF makes use of
        """

        while ( len(DOF) < 6 ):
            DOF = np.append(DOF,0)

        # Rotation matrix of transform matrix
        rotation = R.from_euler(angleSet, [DOF[3], DOF[4], DOF[5]], degrees=True).as_matrix()
        pose = np.eye(4)
        pose[:3, :3] = rotation
        
        # Cartesian coordinates of transform matrix
        pose[:,3] = [ DOF[0], DOF[1], DOF[2], 1 ]

        return pose

    # array xyz[3] covers x-, y-, and z-axis
    def transformMatrix_quaternions(quaternions: float, xyz: float) -> np.matrix:
        """
        Converts any quaternions into a transform matrix including provided xyz-coordinates.
        
        :param str xyz: The xyz-coordinates the transform matrix must have.

        Parameters
        ---
        quaternions : float[4]
            The quaternions in use.

        xyz : double[3]
            The xyz-coordinates the transform matrix must have.
        """

        r = R.from_quat(quaternions).as_matrix()
        T = np.eye(4)
        T[:3, :3] = r
        T[:3,3] = xyz

        return T
    
    def angleAxisSet(TBE: np.matrix, TBS : np.matrix = np.eye(4)) -> list:
        """
        Converts any transform matrix into an axis angle position. E.g. [X, Y, Z, theta].

        Also withs with a start and end transform matrix.

        Parameters:
        ---
        TBE : float[4][4]
            Transform matrix spanning from BASE to END.
        
        TBS : float[4][4]
            Transform matrix spanning from BASE to START. 
            TBS is by default set equal to the identity matrix.
        """

        TSE = np.linalg.inv(TBS) @ TBE
        theta = np.arccos((TSE[0,0] + TSE[1,1] + TSE[2,2] - 1) / 2)
        #Ps = [0, 0, 0, 0]

        Pe = [TSE[0,3], TSE[1,3], TSE[2,3], theta]

        return Pe
