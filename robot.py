import numpy as np
import json
import copy

class robot():

    def __init__(self):
        
        file = open('/home/tuna/Documents/driving/control/Comet/configFiles/orbital_0.0.json', 'r')
        config = json.load(file)
        h = config['support']['height']     #z
        self.curPos = np.array([[0.], [0.], [0.2], [1.]]) #x, y, z, null
        self.curGim = np.array([[0.], [0.]])      #yaw, pitch

        self.mass = config['robot']['mass']
        self.robotRadius = config['robot']['radius']
        self.pulleyRadius = config['robot']['pulleyPitchRadius']
        self.driveRadius = config['robot']['drivePitchRadius']

        self.driveRatio = self.pulleyRadius/self.driveRadius

        self.pulleyID = config['robot']['pulleyID']
        self.thickness = config['robot']['cableThickness']

        #frame conversion robot origin to cable feed outlets
        x = [self.robotRadius*np.cos(np.pi/4),
             self.robotRadius*np.cos(3*np.pi/4),
             self.robotRadius*np.cos(5*np.pi/4),
             self.robotRadius*np.cos(7*np.pi/4),]
        
        y = [self.robotRadius*np.sin(np.pi/4),
             self.robotRadius*np.sin(3*np.pi/4),
             self.robotRadius*np.sin(5*np.pi/4),
             self.robotRadius*np.sin(7*np.pi/4)]
        
        z = [0., 0., 0., 0.]
        #print(z[0])

        #build frame tranformations
        #1st quadrant
        self.robot2Outlet1 = np.array([[1, 0, 0, x[0]],
                                       [0, 1, 0, y[0]],
                                       [0, 0, 1, z[0]],
                                       [0, 0, 0, 1]])      
        #2nd quadrant
        self.robot2Outlet2 = np.array([[1, 0, 0, x[1]],
                                       [0, 1, 0, y[1]],
                                       [0, 0, 1, z[1]],
                                       [0, 0, 0, 1]])      
        #3rd quadrant
        self.robot2Outlet3 = np.array([[1, 0, 0, x[2]],
                                       [0, 1, 0, y[2]],
                                       [0, 0, 1, z[2]],
                                       [0, 0, 0, 1]])      
        #4th quadrant
        self.robot2Outlet4 = np.array([[1, 0, 0, x[3]],
                                       [0, 1, 0, y[3]],
                                       [0, 0, 1, z[3]],
                                       [0, 0, 0, 1]])      

        l = config['support']['length']     #x
        w = config['support']['width']      #y
        h = config['support']['height']     #z

        #position of cable fixed points in table frame
        self.posF1 = np.array([[l/2],  [w/2],  [h]])
        self.posF2 = np.array([[-l/2], [w/2],  [h]])
        self.posF3 = np.array([[-l/2], [-w/2], [h]])
        self.posF4 = np.array([[l/2],  [-w/2], [h]])

        #x, dx, y, dy, z, dz, ###yaw, dyaw, pitch, dpitch
        self.state = np.array([[0.], [0.], [0.], [0.], [0.], [0.]])
        
        #Tor1, Tor2, Tor3, Tor4 [Nm]
        self.u = np.array([[0.], [0.], [0.], [0.]])

        #NOTE: For A and B to be logical, you must calculate the equilibrium torque at the zero
        #       position. Imagine you start from a suspended zero position in the middle of the table
        #       you must calculate the torque required for equilibrium and add that as an offset to your 
        #       your decision vector

        self.A = np.array([[0., 1., 0., 0., 0., 0.], 
                            [0., 0., 0., 0., 0., 0.],
                            [0., 0., 0., 1., 0., 0.],
                            [0., 0., 0., 0., 0., 0.],
                            [0., 0., 0., 0., 0., 1.],
                            [0., 0., 0., 0., 0., 0.]])
        
        self.B = np.array([[0., 0., 0., 0.], 
                            [0., 0., 0., 0.],
                            [0., 0., 0., 0.],
                            [0., 0., 0., 0.], 
                            [0., 0., 0., 0.],
                            [0., 0., 0., 0.]])

    def forwardK(self):
        return
    
    def inverseK(self, desPos=None):

        '''
            if passed position, returns joint angles for motors at desPos (accounting for pulley:drive ratio)
            if position not passed, will return expected current motor angles

            Parameters:
                desPos: 1x3 np.array([x, y, z]) desired position of robot origin in space (0, 0, 0) is home
           
            returns: 
                1x4 numpy array of motorTranslation angles in radians
                3x4 numpy array of unit cable vectors (vectors pointing from feed point on robot to support connection)
                1x4 numpy array of cable lengths
                1x4 numpy array of spool radii (radius to cable on the spool (variable))
                
            AS derivation credit: https://downloads.imagej.net/fiji/snapshots/arc_length.pdf''' 
        
        #TODO: [x] convert from origin robot frame to fixture point frames
        #      [x] update init attributes

        #pulley paramters
        a = self.pulleyID         #starting radius on spool, derived from configuration file
        b = self.thickness           #thickness of the cable

        #if no position passed use current position
        if type(desPos) == type(None):          
            desPos = self.curPos
            
            desPos = np.array([self.state[0], self.state[2], self.state[4], [1.]])
            print('desPos:')
            print(desPos)
            
        #frame transforms (robot origin table frame to outlet points in table frame)
        T1 = self.robot2Outlet1
        T2 = self.robot2Outlet2
        T3 = self.robot2Outlet3
        T4 = self.robot2Outlet4

        #cableVectors:
        C1 = self.posF1 - (T1@desPos)[0:3]
        C2 = self.posF2 - (T2@desPos)[0:3]
        C3 = self.posF3 - (T3@desPos)[0:3]
        C4 = self.posF4 - (T4@desPos)[0:3]
        
        #lengths of each cable
        L1 = np.linalg.norm(C1)
        L2 = np.linalg.norm(C2)
        L3 = np.linalg.norm(C3)
        L4 = np.linalg.norm(C4)
        cableLengths = np.array([L1, L2, L3, L4])

        #unit cable vectors
        cableVectors = np.concatenate((C1/L1, C2/L2, C3/L3, C4/L4), axis = 1)
  
        #radius of cable in spool
        spoolRadii = 2*b*cableLengths/np.pi + a/2

        #driven pulley angles
        th1 = ((2*L1*b/np.pi + a/2) - a)/b   
        th2 = ((2*L2*b/np.pi + a/2) - a)/b 
        th3 = ((2*L3*b/np.pi + a/2) - a)/b 
        th4 = ((2*L4*b/np.pi + a/2) - a)/b          

        #convert to motor output angles, account for drive:driven ratio
        motorTransAngles = np.array([th1, th2, th3, th4])*(self.driveRatio)
      
        # Archimedean spiral defined by:
        #       r = a + b*theta, where a is the starting offset (inner radius) and b describes scaling factor (distance between each spiral)
        #       x(theta) = (a + btheta)cos(theta)
        #       y(theta) = (a + btheta)sin(theta)
        #       
        #       d/d_th(x) = d/d_th(a*cos(th) + b*th*cos(th)) = -a*sin(th) + b(cos(th) - th*sin(th)) = b*cos(th) - sin(th)*(a + b*th)
        #       d/d_th(y) =  ...                                ...                                 = b*sin(th) + cos(th)*(a + b*th)
        
        # Arc length of any curve:
        #       s(theta) = int_(0)^(theta) sqrt(dx(theta)^2 + dy(theta)^2)dtheta
        #       s(th) = int(sqrt((b*cos(th) - sin(th)*(a + b*th))**2 + (b*sin(th) + cos(th)*(a + b*th))**2))

        # L = (pi*(OD - ID))/4*thickness = pi*(2r - a)/4b
        # OD = 4L*thickness/pi + ID =      2r = 4Lb/pi + a      r = 2Lb/pi + a/2
        # r = a + b*th
        # th = (r - a)/b = ((2Lb/pi + a/2) - a)/b
        
        return motorTransAngles, cableVectors, cableLengths, spoolRadii
    
    def updateB(self, cableVecs, spoolRadii, sim=False):

        '''update class obj state array B (time-varying) based on position of robot
                params: 
                    cableVecs:  (3x4) numpy array columns of which are unit vectors from robot feed outlet to support fixture position
                    spoolRadii: (4,)  numpy array of radii at which the cable is currently at in the spool
                    sim:        Bool  If False, updates class variable B, if True, returns updated B matrix wihtout altering class variable'''
        m = self.mass
        dr = self.driveRatio

        #trasnform accounting for mass of robot, drive-pulley ratio, and radii of the spool 
        # (time varying)
        '''torqueUpdate = np.array([[dr/(m*spoolRadii[0]), 0., 0., 0.],
                                 [0., dr/(m*spoolRadii[1]), 0., 0.],
                                 [0., 0., dr/(m*spoolRadii[2]), 0.],
                                 [0., 0., 0., dr/(m*spoolRadii[3])]])'''
        
        #transformation to ensure forces align to ddx not dx in state vector
        # (const)
        '''seperate = np.array([[0, 0, 0],
                             [1, 0, 0],
                             [0, 0, 0],
                             [0, 1, 0],
                             [0, 0, 0],
                             [0, 0, 1]])'''

        #update B
        #self.B = seperate@cableVecs@torqueUpdate

        #NOTE: The only reason I'm updating B this way insteal of matmulling is to potentially avoid issues with autograd later on
        # *I think* if I reassign a new np array at each itter, it will mess up auto diffing, but if I
        # just update the indicies then I think this will work? must check if this is an actual problem
        print(f'cableVec: {cableVecs.shape}')
        print(f'spoolRad: {spoolRadii.shape}')
        if sim:
            B = copy.copy(self.B)
            B[1, :] = cableVecs[0, :]*spoolRadii
            B[3, :] = cableVecs[1, :]*spoolRadii
            B[5, :] = cableVecs[2, :]*spoolRadii
            return B
        
        else:
            self.B[1, :] = cableVecs[0, :]*spoolRadii
            self.B[3, :] = cableVecs[1, :]*spoolRadii
            self.B[5, :] = cableVecs[2, :]*spoolRadii

    def odeStep(self, A, B, x, u, dt=0.01):

        xdot = A*dt@x + B*dt@u - np.array([[0.], [0.], [0.], [0.], [0.], [9.81]])*dt
        x_new = x + xdot
        return x_new
    
    def sim(self, A, B, x, u, dt=0.01, N=200):

        '''simulate forward
        
            PARAMS:
                A: 6x6  numpy array
                B: 6x4  numpy array
                x: 6x1  numpy array
                u: 4x1xN numpy array'''

        x_hist = []
        x_new = self.odeStep(A, B, x, u[i], dt=dt)
        for i in range(1, N):
            x_new = self.odeStep(A, B, x, u[i], dt=dt)
            x_hist.append(x_new)
            B = self.updateB()
            pass

        return


    def staticForces(self):
        return 

    def detectColision(self):
        return
    
if __name__ == "__main__":
    r = robot()