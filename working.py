import numpy as np
from matplotlib import pyplot as plt
import robot
from hermite2D import hermite2D
import pygame
from pygame import locals
import json
import copy



def main(world, robot, traj):

    world.display.fill((255, 255, 255))
    #trackTrajNoDynamics(world, robot, traj)
    N = 2000
    
    A = robot.A
    B = robot.B
    
    #step input
    u = np.ones((4, N))*0.20
    x = robot.state

    #random IC velocities
    x[1] = 1.5      #dx
    x[3] = -2.2     #dy

    dt = 0.001
    t = np.linspace(0, N*dt, N)
    
    #compute simulation
    stateHist = robot.sim(A, B, x, u, N=N)

    for i in range(N):

        robot.state = np.reshape(stateHist[:, i], (6, 1))
        #print(f'robot: {robot.state}')
        _, cableVecs, cableLengths, spoolRadii = robot.inverseK()
        robot.updateB(cableVecs, spoolRadii)

        #only updating this for drawCables()
        robot.curPos[0] = robot.state[0]
        robot.curPos[1] = robot.state[2]
        robot.curPos[2] = robot.state[4]
        
        world.display.fill((255, 255, 255))
        #world.drawTraj(traj, i)
        world.drawRobot(robot)
        world.drawSupport(robot)
        world.drawCables(robot, cableVecs, cableLengths)
        
        pygame.display.update()
        #uncomment line below to save images to make into a gif later on
        #pygame.image.save(world.display, f"/home/tuna/Documents/driving/control/Comet/gifs/10Oct/xy_{i}.png")
        world.fpsClock.tick(N/8)
        # check for quit
        for event in pygame.event.get():
            if event.type == pygame.locals.QUIT:
                pygame.quit() 
                break
    
    fig, ax = plt.subplots(3)
    ax[0].plot(t, stateHist[0, :])
    ax[1].plot(t, stateHist[2, :])
    ax[2].plot(t, stateHist[4, :])
    ax[0].set(ylabel='x [m]')
    ax[1].set(ylabel='y [m]')
    ax[2].set(ylabel='z [m]')
    ax[2].set(xlabel='time [s]')
    plt.show()

    


def trackTrajNoDynamics(world, robot, traj):

    for i, knot in enumerate(traj):
        
        #update robot origin po sition
        robot.curPos[0, 0] = knot[0]
        robot.curPos[1, 0] = knot[1]
        _, cableVecs, cableLengths, spoolRadii = robot.inverseK()

        robot.updateB(cableVecs, spoolRadii)
        print(robot.B)
        
        #draw everything
        world.display.fill((255, 255, 255))
        world.drawTraj(traj, i)
        world.drawRobot(robot)
        world.drawSupport(robot)
        world.drawCables(robot, cableVecs, cableLengths)
        
        pygame.display.update()
        #uncomment line below to save images to make into a gif later on
        #pygame.image.save(world.display, f"/home/tuna/Documents/driving/control/Comet/gifs/10Oct/xy_{i}.png")
        world.fpsClock.tick(30)
        # check for quit
        for event in pygame.event.get():
            if event.type == pygame.locals.QUIT:
                pygame.quit() 
                break

def trackTraj(world, robot, traj):

    for i, knot in enumerate(traj):
        
        #update robot origin po sition
        robot.curPos[0, 0] = knot[0]
        robot.curPos[1, 0] = knot[1]
        _, cableVecs, cableLengths, spoolRadii = robot.inverseK()

        robot.updateB(cableVecs, spoolRadii)
        print(cableVecs)
        #draw everything
        world.display.fill((255, 255, 255))
        world.drawTraj(traj, i)
        world.drawRobot(robot)
        world.drawSupport(robot)
        world.drawCables(robot, cableVecs, cableLengths)
        
        pygame.display.update()
        #uncomment line below to save images to make into a gif later on
        #pygame.image.save(world.display, f"/home/tuna/Documents/driving/control/Comet/gifs/10Oct/xy_{i}.png")
        world.fpsClock.tick(30)
        # check for quit
        for event in pygame.event.get():
            if event.type == pygame.locals.QUIT:
                pygame.quit() 
                break
        
class World():

    def __init__(self):

        pygame.init()

        file = open('/home/tuna/Documents/driving/control/Comet/configFiles/orbital_0.0.json', 'r')
        config = json.load(file)
        self.scaleFactor = 500
        self.width = config['support']['length']*self.scaleFactor
        self.height = config['support']['width']*self.scaleFactor

        self.fpsClock = pygame.time.Clock()
        self.display = pygame.display.set_mode((self.width, self.height))

    def drawRobot(self, robot):

        #draw robot body
        pygame.draw.circle(surface=self.display, 
                           color=(150, 0, 150), 
                           center=(robot.state[0, 0]*self.scaleFactor+world.width/2, robot.state[2, 0]*self.scaleFactor+world.height/2),
                           radius=robot.robotRadius*self.scaleFactor,
                           width=5)

    def drawSupport(self, robot):

        #TODO: [ ] zoom out and show table boundaries with support fixtures

        x0 = robot.posF1[0, 0]*self.scaleFactor*2
        y0 = robot.posF1[1, 0]*self.scaleFactor*2
        x1 = robot.posF1[0, 0]*self.scaleFactor*2
        y1 = robot.posF1[1, 0]*self.scaleFactor*2
        s1 = (x0, y0, x1, y1)
      
        s2 = robot.posF2[0]*self.scaleFactor+self.width/2, robot.posF2[1]*self.scaleFactor+self.height/2
        s3 = robot.posF3[0]*self.scaleFactor+self.width/2, robot.posF3[1]*self.scaleFactor+self.height/2
        s4 = robot.posF4[0]*self.scaleFactor+self.width/2, robot.posF4[1]*self.scaleFactor+self.height/2

        supports = [s1, s2, s3, s4]
        return
       

    def drawCables(self, robot, cableVecs, cableLengths):
        
        #extract transformation mats
        T1 = robot.robot2Outlet1
        T2 = robot.robot2Outlet2
        T3 = robot.robot2Outlet3
        T4 = robot.robot2Outlet4
        transforms = [T1, T2, T3, T4]
        
        for i in range(4):

            #compute in table frame
            start = (transforms[i]@robot.curPos*self.scaleFactor)[0:3]
            end = start + cableVecs[:, i].reshape((3, 1))*cableLengths[i]*self.scaleFactor

            #convert to pygame coordinates
            start = (self.width/2 + start[0, 0], self.height/2 + start[1, 0])
            end = (self.width/2 + end[0,0], self.height/2 + end[1,0])

            #create signed ints for plotting
            start = (int(start[0]), int(start[1]))
            end = (int(end[0]), int(end[1]))

            pygame.draw.line(surface=self.display,
                            color=(100, 100, 100), 
                            start_pos=start,
                            end_pos=end, 
                            width=5)
            
    def drawTraj(self, traj, curStep=None):

        for knot in traj:
           
            pygame.draw.circle(surface=self.display, 
                                color=(200, 200, 200), 
                                center=(knot[0]*self.scaleFactor+world.width/2, 
                                        knot[1]*self.scaleFactor+world.height/2),
                                radius=0.01*self.scaleFactor,
                                width=5)
            
        if curStep:
            pygame.draw.circle(surface=self.display, 
                                color=(20, 20, 20), 
                                center=(traj[curStep][0]*self.scaleFactor+world.width/2, 
                                        traj[curStep][1]*self.scaleFactor+world.height/2),
                                radius=0.01*self.scaleFactor,
                                width=5)

    def drawForces(self, lengths):

        return

        

if __name__ == "__main__":

    robo = robot.Robot()
    world = World()

    knots = [[-0.5, -0.5], 
             [-.7, 0.2],
             [0.4, -0.5], 
             [0.8, 0]]
    
    #flipping y axis (pygame coordinates system)
    for knot in knots:
        for i, k in enumerate(knot): 
            if i != 0:
                k = -k
                knot[i] = k
    
    alphas = [0.5, 0.9, 0.1, 0.5]
    N = 150

    traj = hermite2D(knots, alphas, N)

    main(world, robo, traj)