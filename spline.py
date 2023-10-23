import numpy as np
from matplotlib import pyplot as plt
import copy


class Hermite:

    @staticmethod
    def _2D(knots, alphas, N, visualize=False):

        '''
        This is a slight simplification to the standard hermite method. tangent vectors
        are defined by the previous and next knot points for direction, and a specific alpha 
        value describes the magnitdue of the tangent vector.

        PARAMS:
            knots:      list of 2D coordinate 'knot points' [[x0, y0], ...]
            a:          list of scalar multipiers to apply to velocity profiles [vi, v1, v2, ... vf], as a -> 0, spline -> linear interp
            N:          int, # of dicrete steps
            visualize:  shows a plot
        '''
        if visualize:
            fig = plt.figure(figsize=(10, 10))
            plt.ylim([0, 100])
            plt.xlim([0, 100])
            plt.xlabel('x')
            plt.ylabel('y')
            for i, knot in enumerate(knots):
                plt.plot(knot[0], knot[1], marker="o", markersize=5, color='red')

        vectors = []
        k0 = knots[0]
        k1 = knots[1]

        #intial vector
        mag = ((k0[0]+k1[0])**2 + (k0[1]+k1[1])**2)**0.5                                #magnitude = (x^2 + y^2 + z^2)^0.5
        vec = ((k1[0] - k0[0])*alphas[0]/mag, (k1[1] - k0[1])*alphas[0]/mag)                      #tan.vector, a is a scalar multiplier to detirmine unit vector scaling
        vectors.append(vec)

        #via point tangent vectors (c1)
        for i in range(1, len(knots)-1):
            mag = ((knots[i-1][0]+knots[i+1][0])**2 + (knots[i-1][1]+knots[i+1][1])**2)**0.5   
            vec_tan = ((knots[i-1][0] - knots[i+1][0])*alphas[i]/mag, (knots[i-1][1] - knots[i+1][1])*alphas[i]/mag)
            vectors.append(vec_tan)

        #final vector
        kf = knots[-1]                                                                  #final knot point
        k_ = knots[-2]                                                                  #n-1 knot point

        mag = ((kf[0]+k_[0])**2 + (kf[1]+k_[1])**2)**0.5                                #magnitude = (x^2 + y^2 + z^2)^0.5
        vec = ((k_[0] - kf[0])*alphas[-1]/mag, (k_[1] - kf[1])*alphas[-1]/mag)                    #vector
        vectors.append(vec)

        #print(f'discrete vecs: {vectors}')
        
        #compute points
        pts = []
        U = np.linspace(0, 1, N//(len(knots)-1))                                        #Other ways to do this
        for j in range(len(knots)-1):                                                   

            vecA = copy.copy(vectors[j])
            if j > 0: vecA = (-vecA[0], -vecA[1]) 
                
            vecB = copy.copy(vectors[j+1])
            vecB = (-vecB[0], -vecB[1])                                                 #flipped

            for i in range(len(U)):
                    u = U[i]
                    x = (2*u**3 - 3*u**2 +1)*knots[j][0] + (u**3 - 2*u**2 + u)*vecA[0] + (-2*u**3 + 3*u**2)*knots[j+1][0] + (u**3 - u**2)*vecB[0]
                    y = (2*u**3 - 3*u**2 +1)*knots[j][1] + (u**3 - 2*u**2 + u)*vecA[1] + (-2*u**3 + 3*u**2)*knots[j+1][1] + (u**3 - u**2)*vecB[1]
                    pts.append((x, y))

        if visualize:
            for pt in pts:
                plt.plot(pt[0], pt[1], marker='.', markersize = 1.6, color='black')

            mag = (vectors[0][1]**2 + vectors[0][0]**2)**0.5
            tanX = [knots[0][0], knots[0][0] + (vectors[0][0]/mag)*(alphas[0]/10)]
            tanY = [knots[0][1], knots[0][1] + (vectors[0][1]/mag)*(alphas[0]/10)]
            plt.plot(tanX, tanY, linestyle='-', color='blue')

            for j in range(1, len(knots)-1):
            
                mag = (vectors[j][1]**2 + vectors[j][0]**2)**0.5

                tanX = [knots[j][0] - (vectors[j][0]/mag)*(alphas[j]/10), knots[j][0] + (vectors[j][0]/mag)*(alphas[j]/10)]
                tanY = [knots[j][1] - (vectors[j][1]/mag)*(alphas[j]/10), knots[j][1] + (vectors[j][1]/mag)*(alphas[j]/10)]

                plt.plot(tanX, tanY, linestyle='-', color='blue')

            mag = (vectors[-1][1]**2 + vectors[-1][0]**2)**0.5
            tanX = [knots[-1][0], knots[-1][0] + (vectors[-1][0]/mag)*(alphas[-1]/10)]
            tanY = [knots[-1][1], knots[-1][1] + (vectors[-1][0]/mag)*(alphas[-1]/10)]
            plt.plot(tanX, tanY, linestyle='-', color='blue')
            plt.grid(visible=True)

        return pts
    
    @staticmethod
    def _2Dcontinuous(knots, a, t, computeSpline=False, hermiteVectors=None, N=1):

        '''
        This is a slight simplification to the standard hermite method. Tangent vectors
        are defined by the previous and next knot points for direction, and a specific alpha 
        value describes the magnitdue of the tangent vector.

        In this function, if computeSpline is set to False, this assumes that a list of knots, alphas,
        vectors, and a t will be passed as the hermite spline as already been defined previously. This 
        will evaluate the previously coputed hermite spline at parametre t and return the (x, y, z) tuple.

            If computeSpline==False:
                returns (x, y)
        
        If copmuteSpline == True, you do not need to pass a list of vectors, as these will be computed during
        the solve  of the hermite spline. This will then solve the spline, and then return the vectors that
        can be used to parameterize this spline, as well as the (x, y, z) evaluated at path(t)

            if computeSpline==True:
                return (x, y), hermiteVectors

        PARAMS:
            knots:      list of 3D coordinate 'knot points' [(x0, y0, z0), ...]
            a:          list of scalar multipiers to apply to velocity profiles [vi, v1, v2, ... vf]
            t:          float [0:1] describing the time parameterization of the path
            computeSpline:  Bool, solve for a new spline or use vectors parameterizing previoously computed spline
            hermiteVectors: list of 3D vectors, can be user defined or collected by running this function
                            with computeSpline set to True
            N:          int # of time steps

        Returns:
            (x, y) value of path(t) evaluated at t
        '''

        

        if computeSpline:
            vectors = []
            #intial vector
            k0 = knots[0]
            k1 = knots[1]

            mag = ((k0[0]+k1[0])**2 + (k0[1]+k1[1])**2)**0.5                             #magnitude = (x^2 + y^2 + z^2)^0.5
            vec = ((k1[0] - k0[0])*a[0]/mag, (k1[1] - k0[1])*a[0]/mag)            #vector
            vectors.append(vec)
            
            #via point tangent-tangent vectors (c1)
            for i in range(1, len(knots)-1):
            
                mag = ((knots[i-1][0]+knots[i+1][0])**2 + (knots[i-1][1]+knots[i+1][1])**2)**0.5   
                vec_tan = ((knots[i-1][0] - knots[i+1][0])*a[i]/mag, (knots[i-1][1] - knots[i+1][1])*a[i]/mag)
                vectors.append(vec_tan)

            #final vector
            kf = knots[-1]                                                                                  #final knot point
            k_ = knots[-2]                                                                                  #n-1 knot point

            mag = ((kf[0]+k_[0])**2 + (kf[1]+k_[1])**2)**0.5                             #magnitude = (x^2 + y^2 + z^2)^0.5
            vec = ((k_[0] - kf[0])*a[-1]/mag, (k_[1] - kf[1])*a[-1]/mag)         #vector
            vectors.append(vec)

            #compute path
            pts = []
            U = np.linspace(0, 1, N//(len(knots)-1))
            for j in range(len(knots)-1):                                                   

                vecA = copy.copy(vectors[j])
                if j > 0:
                    vecA = (-vecA[0], -vecA[1]) 
                vecB = copy.copy(vectors[j+1])
                vecB = (-vecB[0], -vecB[1])                                                 #flipped

                for i in range(len(U)):
                        u = U[i]
                        x = (2*u**3 - 3*u**2 +1)*knots[j][0] + (u**3 - 2*u**2 + u)*vecA[0] + (-2*u**3 + 3*u**2)*knots[j+1][0] + (u**3 - u**2)*vecB[0]
                        y = (2*u**3 - 3*u**2 +1)*knots[j][1] + (u**3 - 2*u**2 + u)*vecA[1] + (-2*u**3 + 3*u**2)*knots[j+1][1] + (u**3 - u**2)*vecB[1]
                        
                        pts.append((x, y))

            

            return pts, vectors

        else:

            vectors = hermiteVectors

            #determine which knots t is between
            idx = 0
            for i in range(1, len(knots)-1):
                
                if t >= (i)/(len(knots)-1): 
                    idx += 1 
                else:
                    break
            
            #remap t from 0-1 global to 0-1 within the knots
            min = (idx)/(len(knots)-1)
            max = (idx+1)/(len(knots)-1)
            u = (t - min)/(max - min)

            vecA = copy.copy(vectors[idx])
            if idx > 0:
                vecA = (-vecA[0], -vecA[1]) 
            vecB = copy.copy(vectors[idx+1])
            vecB = (-vecB[0], -vecB[1])   

            x = (2*u**3 - 3*u**2 +1)*knots[idx][0] + (u**3 - 2*u**2 + u)*vecA[0] + (-2*u**3 + 3*u**2)*knots[idx+1][0] + (u**3 - u**2)*vecB[0]
            y = (2*u**3 - 3*u**2 +1)*knots[idx][1] + (u**3 - 2*u**2 + u)*vecA[1] + (-2*u**3 + 3*u**2)*knots[idx+1][1] + (u**3 - u**2)*vecB[1]
             
            return (x, y)
    

    @staticmethod
    def _3D(knots, a, N):

        '''
        This is a slight simplification to the standard hermite method. tangent vectors
        are defined by the previous and next knot points for direction, and a specific alpha 
        value describes the magnitdue of the tangent vector.

        PARAMS:
            knots:  list of 3D coordinate 'knot points' [[x0, y0, z0], ...]
            a:      list of scalar multipiers to apply to velocity profiles [vi, v1, v2, ... vf]
            N:      int, # of dicrete steps

        Returns:
            3xN list of discrete points
        '''
        vectors = []
        
        #intial vector
        k0 = knots[0]
        k1 = knots[1]
        
        mag = ((k0[0]+k1[0])**2 + (k0[1]+k1[1])**2 + (k0[2]+k1[2])**2)**0.5                             #magnitude = (x^2 + y^2 + z^2)^0.5
        vec = ((k1[0] - k0[0])*a[0]/mag, (k1[1] - k0[1])*a[0]/mag, (k1[2] - k0[2])*a[0]/mag)            #vector
        vectors.append(vec)

        #via point tangent-tangent vectors (c1)
        for i in range(1, len(knots)-1):
            
            mag = ((knots[i-1][0]+knots[i+1][0])**2 + (knots[i-1][1]+knots[i+1][1])**2 + (knots[i-1][2]+knots[i+1][2])**2)**0.5   
            vec_tan = ((knots[i-1][0] - knots[i+1][0])*a[i]/mag, (knots[i-1][1] - knots[i+1][1])*a[i]/mag, (knots[i-1][2] - knots[i+1][2])*a[i]/mag)
            vectors.append(vec_tan)

        #final vector
        kf = knots[-1]                                                                                  #final knot point
        k_ = knots[-2]                                                                                  #n-1 knot point

        mag = ((kf[0]+k_[0])**2 + (kf[1]+k_[1])**2 + (kf[2]+k_[2])**2)**0.5                             #magnitude = (x^2 + y^2 + z^2)^0.5
        vec = ((k_[0] - kf[0])*a[-1]/mag, (k_[1] - kf[1])*a[-1]/mag, (k_[2] - kf[2])*a[-1]/mag)         #vector
        vectors.append(vec)

        #compute path
        pts = []
        U = np.linspace(0, 1, N//(len(knots)-1))
        for j in range(len(knots)-1):                                                   

            vecA = copy.copy(vectors[j])
            if j > 0:
                vecA = (-vecA[0], -vecA[1], -vecA[2]) 
            vecB = copy.copy(vectors[j+1])
            vecB = (-vecB[0], -vecB[1], -vecB[2])                                                 #flipped

            for i in range(len(U)):
                    u = U[i]
                    x = (2*u**3 - 3*u**2 +1)*knots[j][0] + (u**3 - 2*u**2 + u)*vecA[0] + (-2*u**3 + 3*u**2)*knots[j+1][0] + (u**3 - u**2)*vecB[0]
                    y = (2*u**3 - 3*u**2 +1)*knots[j][1] + (u**3 - 2*u**2 + u)*vecA[1] + (-2*u**3 + 3*u**2)*knots[j+1][1] + (u**3 - u**2)*vecB[1]
                    z = (2*u**3 - 3*u**2 +1)*knots[j][2] + (u**3 - 2*u**2 + u)*vecA[2] + (-2*u**3 + 3*u**2)*knots[j+1][2] + (u**3 - u**2)*vecB[2]
                    pts.append((x, y, z))

        return pts
    

    @staticmethod
    def _3Dcontinuous(knots, a, t, computeSpline=False, hermiteVectors=None, N=1):

        '''
        This is a slight simplification to the standard hermite method. Tangent vectors
        are defined by the previous and next knot points for direction, and a specific alpha 
        value describes the magnitdue of the tangent vector.

        In this function, if computeSpline is set to False, this assumes that a list of knots, alphas,
        vectors, and a t will be passed as the hermite spline as already been defined previously. This 
        will evaluate the previously coputed hermite spline at parametre t and return the (x, y, z) tuple.

            If computeSpline==False:
                returns (x, y, z)
        
        If copmuteSpline == True, you do not need to pass a list of vectors, as these will be computed during
        the solve  of the hermite spline. This will then solve the spline, and then return the vectors that
        can be used to parameterize this spline, as well as the (x, y, z) evaluated at path(t)

            if computeSpline==True:
                return (x, y, z), hermiteVectors

        PARAMS:
            knots:      list of 3D coordinate 'knot points' [(x0, y0, z0), ...]
            a:          list of scalar multipiers to apply to velocity profiles [vi, v1, v2, ... vf]
            t:          float [0:1] describing the time parameterization of the path
            computeSpline:  Bool, solve for a new spline or use vectors parameterizing previoously computed spline
            hermiteVectors: list of 3D vectors, can be user defined or collected by running this function
                            with computeSpline set to True
            N:          int # of time steps

        Returns:
            (x, y, z) value of path(t) evaluated at t
        '''

        if computeSpline:
            vectors = []
            #intial vector
            k0 = knots[0]
            k1 = knots[1]

            mag = ((k0[0]+k1[0])**2 + (k0[1]+k1[1])**2 + (k0[2]+k1[2])**2)**0.5                             #magnitude = (x^2 + y^2 + z^2)^0.5
            vec = ((k1[0] - k0[0])*a[0]/mag, (k1[1] - k0[1])*a[0]/mag, (k1[2] - k0[2])*a[0]/mag)            #vector
            vectors.append(vec)
            
            #via point tangent-tangent vectors (c1)
            for i in range(1, len(knots)-1):
            
                mag = ((knots[i-1][0]+knots[i+1][0])**2 + (knots[i-1][1]+knots[i+1][1])**2 + (knots[i-1][2]+knots[i+1][2])**2)**0.5   
                vec_tan = ((knots[i-1][0] - knots[i+1][0])*a[i]/mag, (knots[i-1][1] - knots[i+1][1])*a[i]/mag, (knots[i-1][2] - knots[i+1][2])*a[i]/mag)
                vectors.append(vec_tan)

            #final vector
            kf = knots[-1]                                                                                  #final knot point
            k_ = knots[-2]                                                                                  #n-1 knot point

            mag = ((kf[0]+k_[0])**2 + (kf[1]+k_[1])**2 + (kf[2]+k_[2])**2)**0.5                             #magnitude = (x^2 + y^2 + z^2)^0.5
            vec = ((k_[0] - kf[0])*a[-1]/mag, (k_[1] - kf[1])*a[-1]/mag, (k_[2] - kf[2])*a[-1]/mag)         #vector
            vectors.append(vec)

            #compute path
            pts = []
            U = np.linspace(0, 1, N//(len(knots)-1))
            for j in range(len(knots)-1):                                                   

                vecA = copy.copy(vectors[j])
                if j > 0:
                    vecA = (-vecA[0], -vecA[1], -vecA[2]) 
                vecB = copy.copy(vectors[j+1])
                vecB = (-vecB[0], -vecB[1], -vecB[2])                                                 #flipped

                for i in range(len(U)):
                        u = U[i]
                        x = (2*u**3 - 3*u**2 +1)*knots[j][0] + (u**3 - 2*u**2 + u)*vecA[0] + (-2*u**3 + 3*u**2)*knots[j+1][0] + (u**3 - u**2)*vecB[0]
                        y = (2*u**3 - 3*u**2 +1)*knots[j][1] + (u**3 - 2*u**2 + u)*vecA[1] + (-2*u**3 + 3*u**2)*knots[j+1][1] + (u**3 - u**2)*vecB[1]
                        z = (2*u**3 - 3*u**2 +1)*knots[j][2] + (u**3 - 2*u**2 + u)*vecA[2] + (-2*u**3 + 3*u**2)*knots[j+1][2] + (u**3 - u**2)*vecB[2]
                        pts.append((x, y, z))

            return pts

        else:
            vectors = hermiteVectors

            #determine which knots t is between
            idx = 0
            for i in range(1, len(knots)-1):
                
                if t >= (i)/(len(knots)-1): 
                    idx += 1 
                else:
                    break
            
            #remap t from 0-1 global to 0-1 within the knots
            min = (idx)/(len(knots)-1)
            max = (idx+1)/(len(knots)-1)
            u = (t - min)/(max - min)

            x = (2*u**3 - 3*u**2 +1)*knots[idx][0] + (u**3 - 2*u**2 + u)*vecA[0] + (-2*u**3 + 3*u**2)*knots[idx+1][0] + (u**3 - u**2)*vecB[0]
            y = (2*u**3 - 3*u**2 +1)*knots[idx][1] + (u**3 - 2*u**2 + u)*vecA[1] + (-2*u**3 + 3*u**2)*knots[idx+1][1] + (u**3 - u**2)*vecB[1]
            z = (2*u**3 - 3*u**2 +1)*knots[idx][2] + (u**3 - 2*u**2 + u)*vecA[2] + (-2*u**3 + 3*u**2)*knots[idx+1][2] + (u**3 - u**2)*vecB[2]
            
            return (x, y, z)
    
