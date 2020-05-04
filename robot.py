import numpy as np
import pygame
class Brittlestar:


    def __init__(self,*,x,y,o,vel_t):
        self.actions = [0,1,2,3,4]
        self.x = x
        self.y = y
        self.o = o
        self.vel_t = vel_t
        totalAng = 2*np.pi
        self.legNum = len(self.actions)
        self.legAng = totalAng/self.legNum
        self.noise = 0
        self.last_action  = 0
        self.sprite = [pygame.image.load('leg_0.png'),pygame.image.load('leg_1.png'),pygame.image.load('leg_2.png'),pygame.image.load('leg_3.png'),pygame.image.load('leg_4.png')]
    
    def move(self, action):
        self.x,self.y = self.peek(action)
        self.x = self.x + self.vel_t*self.noise*np.random.uniform(-1,1,1)
        self.y = self.y + self.vel_t*self.noise*np.random.uniform(-1,1,1)
        self.o = self.o + self.noise*np.random.uniform(-1,1,1)


    def peek(self,action, x = None, y = None):
        if x is None or y is None:
            x = self.x
            y = self.y
        x_new = int(x + self.vel_t * np.cos(self.o + action*self.legAng))
        y_new = int(y + self.vel_t * -np.sin(self.o + action*self.legAng))
        return (x_new,y_new)

    def seek(self, goal):
        dists = np.array(range(len(self.actions)))
        for i,action in enumerate(self.actions):
            x,y =  self.peek(action)
            dists[i] = np.linalg.norm([goal.x-x,goal.y-y])
        best_action = np.argmin(dists)
        self.move(best_action)
        self.last_action = best_action
        return self.x, self.y