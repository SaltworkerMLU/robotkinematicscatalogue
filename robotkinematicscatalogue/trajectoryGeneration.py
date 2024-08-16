from robotkinematicscatalogue.source import *

class trajectoryGeneration:
    """
    # This class does trajectory generation
    """
    def moveJ_tf(P0 : list, Pf : list, v_max : float, a_max : float) -> float:
        """
        # Assuming a parabolic blend is used
        Calculate the execution time of specified joint movement.

        Parameters:
            P0 : float[]
                The start position of the movement. Position P(0)
            
            Pf : float[]
                The end position of the movement. Position P(f)
            
            v_max : float
                The maximum velocity of the joint movement.
            
            a_max : float
                The maximum acceleration of the joint movement.
        """

        tf_a = max(np.sqrt(abs(4 * (Pf - P0) / a_max))) # Execution time assuming max acceleration reached
        tf_v = max(abs((Pf - P0) / v_max)) # Execution time assuming max velocity reached
        tb = v_max / a_max # Calculate border time tb assuming constant acceleration.

        if 2 * tb > tf_a: # If max acceleration is never reached
            tf = tf_a
        else: # If max acceleration IS reached
            tf = tf_v + tb
        print(f"\nExecution time of moveJ(): {tf}s")
        return tf

    def moveL_tf(TBS : np.matrix, TBE : np.matrix, v_max : float, a_max : float) -> float:
        """
        # Assuming a parabolic blend is used
        Calculate the execution time of specified linear movement.

        Parameters:
            TBS : float[4][4]
                The transform matrix of the start position
            
            TBE : float[4][4]
                The transform matrix of the end position
            
            v_max : float
                The maximum velocity of the joint movement.
            
            a_max : float
                The maximum acceleration of the joint movement.
        """

        # Conversion to angle axis representation
        TSE = np.linalg.inv(TBS) @ TBE
        theta = np.arccos( (TSE[0][0] + TSE[1][1] + TSE[2][2] - 1) / 2)
        Ps = np.zeros(4)
        Pe = [TSE[0][3], TSE[1][3], TSE[2][3], theta]

        tf_a = np.sqrt(4 * np.linalg.norm(Pe - Ps) / a_max) # Execution time assuming max acceleration reached
        tf_v = np.linalg.norm(Pe - Ps) / v_max # Execution time assuming max velocity reached
        tb = v_max / a_max # Calculate border time tb assuming constant acceleration.

        if 2 * tb > tf_a: # If max acceleration is never reached
            tf = tf_a
        else: # If max acceleration IS reached
            tf = tf_v + tb
        print(f"\nExecution time of moveJ(): {tf}s")
        return tf
    
    def cubicPolynomial(Ps : list, Pe : list, tf : list, dPs : list = np.zeros(6), dPe : list = np.zeros(6)) -> list:
        """
        Calculates the cubic polynomial of specified movement from @Ps to @Pe.

        Parameters
        ---

        Ps : float[]
            Start position of each joint
        
        Pe : float[]
            End position of each joint
        
        tf : float[]
            Execution time of each joint
        
        dPs : float[]
            Start velocity of each joint
        
        dPe : float[]
            End velocity of each joint

        Returns
        ---
        cubic : sym.Symbol( float[] )
            Return cubic polynomial of each joint. 
            
            Beware of the complexity of the datatype when using said return value.
        """
        t = sym.Symbol('t') # Symbolic 't' used to define mathematical function
        cubicCoefficients = np.zeros([len(Ps),4]) # The cubic polynomial has 4 parameters

        # Define matrix coefficients of cubic polynomial
        for i in range(len(Ps)): #range(len(Ps))
            cubicCoefficients[i][0] = Ps[i]
            cubicCoefficients[i][1] = dPs[i]
            cubicCoefficients[i][2] = 3/tf[i]**2 * (Pe[i]-Ps[i]) - 2/tf[i] * dPs[i] - 1/tf[i] * dPe[i]
            cubicCoefficients[i][3] = - 2/tf[i]**3 * (Pe[i]-Ps[i]) + 1/tf[i]**2 * (dPe[i] + dPs[i])

        cubic = cubicCoefficients @ np.transpose([1, t, t**2, t**3]) # Matrix-vector product

        # Print result for debugging purposes
        print("\nCUBIC POLYNOMIAL:")
        for i in range(len(cubic)): #range(len(solution))
            print(cubic[i])

        return cubic
    
    def cubicPolynomialPlot(Ps : list, Pe : list, tf : list, dPs : list = np.zeros(6), dPe : list = np.zeros(6)) -> None:
        """
        Plots the cubic polynomial of specified movement from @Ps to @Pe.

        Parameters
        ---

        Ps : float[]
            Start position of each joint
        
        Pe : float[]
            End position of each joint
        
        tf : float[]
            Execution time of each joint
        
        dPs : float[]
            Start velocity of each joint
        
        dPe : float[]
            End velocity of each joint
        """
        # MatPlotLib plotting
        figure, axis = plt.subplots(2, 3)
        figure.suptitle("Cubic Polynomial")

        for i in range(len(Ps)):
            t = np.linspace(0, tf[i], 400)

            P1 = Ps[i]
            P2 = dPs[i]
            P3 = 3/tf[i]**2 * (Pe[i]-Ps[i]) - 2/tf[i] * dPs[i] - 1/tf[i] * dPe[i]
            P4 = - 2/tf[i]**3 * (Pe[i]-Ps[i]) + 1/tf[i]**2 * (dPe[i] + dPs[i])
            P = P1 + P2 * t + P3 * t**2 + P4 * t**3

            axis[i//3,i%3].plot(t, P, 'r') # plotting t, a separately 

            axis[i//3,i%3].set(xlabel='time (s)', ylabel='degrees')
            axis[i//3,i%3].set_title('Joint ' + str(i+1)) # Name of each plot
            axis[i//3,i%3].set_xlim([0, tf[i]])
            #axis[i//3,i%3].set_ylim([Ps[i], Pe[i]])
        
        plt.show()

    def cubicPolynomialPoint(Ps : list, Pe : list, tf : list, t: list, dPs : list = np.zeros(6), dPe : list = np.zeros(6)) -> list:
        """
        Plots the cubic polynomial of specified movement from @Ps to @Pe.

        Parameters
        ---

        Ps : float[]
            Start position of each joint
        
        Pe : float[]
            End position of each joint
        
        tf : float[]
            Execution time of each joint
        
        t : float[]
            Calculate cubic polynomial of said time @t.
        
        dPs : float[]
            Start velocity of each joint
        
        dPe : float[]
            End velocity for each joint

        Returns
        ---
        point : float[len(Ps)]
            A joint value of each joint.
        """
        point = np.zeros(len(Ps))
        cubicCoefficients = np.zeros([len(Ps), 4])

        print(f"\nCUBIC POLYNOMIAL f({t})\n")
        for i in range(len(Ps)):
            cubicCoefficients[i][0] = Ps[i]
            cubicCoefficients[i][1] = dPs[i]
            cubicCoefficients[i][2] = 3/tf[i]**2 * (Pe[i]-Ps[i]) - 2/tf[i] * dPs[i] - 1/tf[i] * dPe[i]
            cubicCoefficients[i][3] = - 2/tf[i]**3 * (Pe[i]-Ps[i]) + 1/tf[i]**2 * (dPe[i] + dPs[i])

            point[i] = cubicCoefficients[i][0] + cubicCoefficients[i][1] * t[i] + cubicCoefficients[i][2] * t[i]**2 + cubicCoefficients[i][3] * t[i]**3
        
        print(point)

        return point

    def parabolicBlend(Ps : list, Pe : list, tf : list, a : list = False) -> np.matrix:
        """
        Calculates the parabolic blend of specified movement from @Ps to @Pe.

        Parameters
        ---

        Ps : float[]
            Start position of each joint
        
        Pe : float[]
            End position of each joint
        
        tf : float[]
            Execution time of each joint
        
        a : float[] = False
            If let to be default value, the maximum acceleration is used.
            
            Otherwise, if acceleration is specified, said acceleration will be used.
            
            MUST CONTAIN ONLY POSITIVE VALUES!!!
        
        Returns
        ---
        parabolic : sym.Symbol( float[:,3] )
            Returns all 3 segments of the parabolic blend of each joint.

            Beware of the complexity of the datatype when using said return value.
        """

        # By default, use maximum acceleration
        if (a == False):
            a = (4 * (Pe - Ps)) / tf**2
        else: # Otherwise, used specified acceleration
            a = np.full(len(Ps), a)
            for i in range(len(Ps)):
                # Should acceleration be positive or negative?
                if (Pe[i] < Ps[i]):
                    a[i] = -a[i]

                # Are the specified accelerations below max acceleration?
                if not( (abs(a[i])>= 4 * (Pe[i] - Ps[i]) / tf**2) ):
                    print("no")
                    return

        # Calculate border time tb assuming parabolic blend is used.
        tb = tf/2 - abs( np.sqrt(a**2 * tf**2 - 4*a*(Pe-Ps) )/(2*a) )

        # If any border time is invalid, set it to the maximum value of tb
        for i in range(len(Ps)):
            if np.isnan(tb[i]):
                tb[i] = max(tb)


        thetab = Ps + 1/2 * a * tb**2

        t = sym.Symbol('t') # Symbolic 't' used to define mathematical function
        parabolic = np.full([len(Ps), 3], None) # By not defining variable type, "expressions" can be used.

        # The mathematical functions describing each segment of the parabolic blend.
        for i in range(len(Ps)):
            parabolic[i][0] = Ps[i] + 1/2 * a[i] * t**2
            parabolic[i][1] = thetab[i] + a[i] * tb[i] * (t-tb[i])
            parabolic[i][2] = Pe[i] - 1/2 * a[i] * (tf - t)**2

        # Print result for debugging purposes
        print("\nPARABOLIC BLEND:")
        for i in range(len(Ps)): #range(len(solution))
            for j in range(3):
                print(parabolic[i][j], end="\t")
            print("\n")
        return parabolic

    def parabolicBlendPlot(Ps : list, Pe : list, tf : list, a : list = False) -> None:
        """
        Plots the parabolic blend of specified movement from @Ps to @Pe.

        Parameters
        ---

        Ps : float[]
            Start position of each joint
        
        Pe : float[]
            End position of each joint
        
        tf : float[]
            Execution time of each joint
        
        a : float[] = False
            If let to be default value, the maximum acceleration is used.
            
            Otherwise, if acceleration is specified, said acceleration will be used.
            
            MUST CONTAIN ONLY POSITIVE VALUES!!!
        """

        if (a == False):
            a = (4 * (Pe - Ps)) / tf**2
        else:
            a = np.full(len(Ps), a)
            for i in range(len(Ps)):
                if (Pe[i] < Ps[i]):
                    a[i] = -a[i]

                if not( (abs(a[i])>= 4 * (Pe[i] - Ps[i]) / tf**2) ):
                    print("no")
                    return

        tb = tf/2 - abs( np.sqrt(a**2 * tf**2 - 4*a*(Pe-Ps) )/(2*a) )
        for i in range(len(Ps)):
            if np.isnan(tb[i]):
                tb[i] = max(tb)

        thetab = Ps + 1/2 * a * tb**2

        # MatPlotLib plotting
        figure, axis = plt.subplots(2, 3)
        figure.suptitle("Parabolic Blend")

        for i in range(len(Ps)):
            t1 = np.linspace(0, tb[i], 400)
            t2 = np.linspace(tb[i], tf-tb[i], 400)
            t3 = np.linspace(tf-tb[i], tf, 400)

            p1 = Ps[i] + 1/2 * a[i] * t1**2
            p2 = thetab[i] + a[i] * tb[i] * (t2-tb[i])
            p3 = Pe[i] - 1/2 * a[i] * (tf - t3)**2

            axis[i//3,i%3].plot(t1, p1, 'r') # plotting t, a separately 
            axis[i//3,i%3].plot(t2, p2, 'b') # plotting t, b separately 
            axis[i//3,i%3].plot(t3, p3, 'g') # plotting t, c separately 

            axis[i//3,i%3].set(xlabel='time (s)', ylabel='degrees')
            axis[i//3,i%3].set_title('Joint ' + str(i+1)) # Name of each plot
            axis[i//3,i%3].set_xlim([0, tf])
            #axis[i//3,i%3].set_ylim([Ps[i], Pe[i]])
        
        plt.show()

    def parabolicBlendPoint(Ps : list, Pe : list, tf : list, t : float, a : list = False) -> list:
        """
        Calculates a point on the parabolic blend of specified movement from @Ps to @Pe.

        Parameters
        ---

        Ps : float[]
            Start position of each joint
        
        Pe : float[]
            End position of each joint
        
        tf : float[]
            Execution time of each joint
        
        a : float[] = False
            If let to be default value, the maximum acceleration is used.
            
            Otherwise, if acceleration is specified, said acceleration will be used.
            
            MUST CONTAIN ONLY POSITIVE VALUES!!!
        
        Returns
        ---

        point : float[]
            Returns joint value of each joint.
        """

        if (a == False):
            a = (4 * (Pe - Ps)) / tf**2
        else:
            a = np.full(len(Ps), a)
            for i in range(len(Ps)):
                if (Pe[i] < Ps[i]):
                    a[i] = -a[i]

                if not( (abs(a[i])>= 4 * (Pe[i] - Ps[i]) / tf**2) ):
                    print("no")
                    return

        tb = tf/2 - abs( np.sqrt(a**2 * tf**2 - 4*a*(Pe-Ps) )/(2*a) )
        for i in range(len(Ps)):
            if np.isnan(tb[i]):
                tb[i] = max(tb)

        thetab = Ps + 1/2 * a * tb**2

        point = np.zeros(len(Ps))

        print(f"\nPARABOLIC BLEND f({t}) = ")
        for i in range(len(Ps)):
            if t < tb[i]:
                point[i] = Ps[i] + 1/2 * a[i] * t**2

            if tb[i] < t and t < tf - tb[i]:
                point[i] = thetab[i] + a[i] * tb[i] * (t-tb[i])

            if tf - tb[i] < t:
                point[i] = Pe[i] - 1/2 * a[i] * (tf - t)**2
            
            print(point[i])
        
        return point

    def profile(f : sym.Symbol, point : list = False) -> np.matrix:
        """
        Calculate kinematic profile of said function. A "profile" consists of:
        - Position -> The function f
        - Velocity -> Its derivative f'
        - Acceleration -> Its double derivative f''

        Parameters
        ---

        f : sym.Symbol( float[] )
            The mathematical function to get kinematic profile of.
        
        point : specified point in time of said mathematical function.

        Returns
        ---
        sym.Matrix(kinematicProfile) : sym.Symbol( float[] )
            The kinematic profile of each used joint.
        """

        f = sym.Matrix(f)
        # Rounding "ridiculous" near-zero number up to float
        for a in sym.preorder_traversal(f):
            if isinstance(a, sym.Float):
                f = f.subs(a, round(a,5))

        df = sym.zeros(len(f), 1)
        ddf = sym.zeros(len(f), 1)

        for i in range(len(f)):
            df[i] = sym.diff(f[i], sym.Symbol('t'))
            ddf[i] = sym.diff(df[i], sym.Symbol('t'))

        if (point != False):
            for i in range(len(f)):
                f[i] = f[i].subs(sym.Symbol('t'), point[i])
                df[i] = df[i].subs(sym.Symbol('t'), point[i])
                ddf[i] = ddf[i].subs(sym.Symbol('t'), point[i])

        kinematicProfile = [f, df, ddf]
        sym.pprint(kinematicProfile)

        return kinematicProfile