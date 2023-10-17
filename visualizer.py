import Kinematics
import numpy as np
import pygame


def main(robot):
    return

if __name__ == "__main__":

    robot = Kinematics.robot()



'''

This is a cleaned up version of planarBot

working:
    - forward kinematics using DH table
    - inverse kinematics with N-R and armijo
    - linear trajectory from pt1 to pt2

next itteration:
    - augmented lagrangian to constrain joint limits and velocities
    - must work out dynamics model for planar bot theta_ddot
    - pass-through singularity detector

'''

'''
robot state vector = [l1.theta
                      l1.thetaDot
                      l2.theta
                      ...
                      ]'''

class World():

    def __init__(self):
        pygame.init()
 
        self.width = 750    #screen dimensions
        self.height = 750

        self.start = None   #(x, y) in world frame
        self.goal = None    #(x, y) in world frame

        self.fpsClock = pygame.time.Clock()
        self.display = pygame.display.set_mode((self.width, self.height))

    def drawRobot(self, robot):

        #draw robot base triangle
        pygame.draw.polygon(surface=self.display,
                        color=(0,0,0),
                        points=[(robot.baseX - 20, robot.baseY + 30),
                                (robot.baseX, robot.baseY),
                                (robot.baseX + 20, robot.baseY + 30)])
        
        for num, link in enumerate(robot.links):

            #draw links
            pygame.draw.line(surface=self.display, 
                            color=(100, 100, 100), 
                            start_pos=robot.robot2World(link.start), 
                            end_pos=robot.robot2World(link.end),
                            width = 5)
        
            #draw 'joints'
            pygame.draw.circle(surface=self.display,
                            center=robot.robot2World(link.start),
                            color=(0, 0, 0),
                            radius= 6,
                            width=15)
        
        #draw end effector
        pygame.draw.circle(surface=self.display,
                        center=robot.robot2World(link.end),
                        color=(200, 0, 200),
                        radius= 5,
                        width=15)
            
    def drawPoint(self, point, color, label=None, size=5):

        pygame.draw.circle(surface=world.display,
                           color= color,
                           center= point, 
                           radius= size)
        
        if label:
            font = pygame.font.Font('freesansbold.ttf', 15)
            text = font.render(label, True, (0, 0, 0), None)
            textR = text.get_rect()
            textR.center = (point[0], point[1] - 15)
            self.display.blit(text, textR)
        
       

class Robot():

    def __init__(self):
        self.baseX = 0              #x-coord in world frame
        self.baseY = 0              #y-coord in world frame
        self.start = None           #(x, y) in robot frame
        self.goal = None            #(x, y) in robot frame
        self.links = []             #storing links
        self.states = np.array([])  #[theta1, theta1Dot, theta2, ...] in robot frame
        self.DH = []                #storing kinematic matrices

    def addLink(self, link):

        if len(self.links) == 0:
            link.start = robot.world2robot((robot.baseX, robot.baseY)) #robot
            
        self.links.append(link)
        self.states = np.append(self.states, np.array([-np.pi/4, 0]))  #init configuration


    def linearTraj(self, pointA, pointB, steps):

        '''Returns list len(steps) of points along a line between A and B
            A, B:   tuples in robot frame
            steps:  int: (0, inf)
            traj:   [(x0, y0), (x1, y1),...] in robot frame'''
        
        traj = []

        vec = [pointB[0] - pointA[0], pointB[1] - pointA[1]]    #vector A to B
        mag = (vec[0]**2 + vec[1]**2)**0.5                      #|vec|
        u = [vec[0]/mag, vec[1]/mag]                            #unit vector
       
        step = mag/steps
     

        point = copy.copy([pointA[0], pointA[1]])
        traj.append((point[0], point[1]))                       #starting position

        for i in range(steps):

            point[0] += u[0]*step
            point[1] += u[1]*step
       
            traj.append((point[0], point[1]))
            
        return traj

   

class Link():

    def __init__(self):
        self.length = 0
        self.mass = 0
        self.theta = 0          #float in world frame for drawing the robot
        self.start = None       #(x, y) robot frame
        self.end = None         #(x, y) robot frame


    def setImage(self):
        self.pyLink = pygame.Surface((self.length, 15), pygame.SRCALPHA, 32)
        self.pyLink.fill((50, 50, 50, 200))

    def getRelativeEndPoint(self):

        endX = self.length * np.cos(self.theta)
        endY = self.length * np.sin(self.theta)

        return (endX, endY)




            

def main(world, robot):

    c_space = 0

    #detirmine c-space radius (assumes no joint limits, just to show reachable space)
    for link in robot.links:
        c_space += link.length

    #define arbtrary color vars for drawing functions
    black = (0,0,0)
    white = (255, 255, 255)
    red = (240, 0, 0)
    green = (0, 240, 0)
    blue = (0, 0, 240)
    lightGrey = (230, 230, 230)
    darkGrey = (80, 80, 80)
    
    
    robot.forwardKinematics()                   #solve (x, y) positions for each joint in robot frame
    
    robot.start = robot.links[-1].end
    traj = robot.linearTraj(pointA=robot.start, pointB=robot.goal, steps=50)     #generate trajectory from pt1 - pt2

    for i in range(len(traj)-1):

        robot.start = traj[i]                   #update starting position
        robot.goal = traj[i+1]                  #update goal position
        
        x, x_hist = solveIK(robot, tol=1e-4)    #solve IK problem w NR method + armijo
        robot.states[0] = x[0]                  #update states
        robot.states[2] = x[1]
        robot.states[4] = x[2]
        robot.forwardKinematics()               #update joints

        world.display.fill((255, 255, 255))

        #draw c-Space
        pygame.draw.circle(surface=world.display,
                           color=lightGrey,
                           center=(robot.baseX, robot.baseY),
                           radius=c_space
            )
        
        #draw trajectory steps
        for j in range(len(traj)):
            worldPoint = robot.robot2World(traj[j])
            world.drawPoint(worldPoint, color=darkGrey, size=2)

        #draw objective points
        world.drawPoint(world.start, color=green, label='Start')
        world.drawPoint(world.goal, color=red, label='Goal')

        #draw robot representation
        world.drawRobot(robot)

        ##  DNE  ##
        pygame.display.update()
        #uncomment line below to save images to make into a gif later on
        #pygame.image.save(world.display, f"/home/tuna/Documents/driving/control/planarBot/cost_gif/varianceNR{i}.png")
        world.fpsClock.tick(10)
        # check for quit
        for event in pygame.event.get():
            if event.type == pygame.locals.QUIT:
                pygame.quit() 
                break

                            

if __name__ == "__main__":

    world = World()

    robot = Robot()
    robot.baseX = world.width/2
    robot.baseY = world.height/2

    l1 = Link()
    l1.length = 100.
    l1.theta = -np.pi/12

    l2 = Link()
    l2.length = 100.
    l2.theta = -np.pi/2

    l3 = Link()
    l3.length = 100.
    l3.theta = np.pi/4

    robot.addLink(l1)
    robot.addLink(l2)
    robot.addLink(l3)

    #set up + solve inital robot configuration
    robot.start = robot.forwardKinematics(ret=True) #robot frame
    robot.goal = (240, -160)                        #robot frame (set this to be your actual desired start)
    
    x, _ = solveIK(robot, tol=1e-4)                 #solve
    
    robot.states[0] = x[0]                          #update states
    robot.states[2] = x[1]
    robot.states[4] = x[2]

    robot.start = robot.forwardKinematics(ret=True) #robot frame (actual desired start)
    robot.goal = (-295, 0)                          #robot frame (actual goal for demo)
    

    world.start = robot.robot2World(robot.start, ret= 'tuple')  #world frame
    world.goal = robot.robot2World(robot.goal, ret= 'tuple')    #world frame

    #visualize demo
    main(world, robot)