import pygame
import numpy as np
import random
from robot import Brittlestar


class PosNode:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.path_x = []
        self.path_y = []

def create_graph(back_width,back_height,robot,goal):
    graph = {}
    min_dist = np.linalg.norm([robot.x-goal.x,robot.y-goal.y])
    i = 0
    graph[i] = [PosNode(robot.x,robot.y)]
    while min_dist > robot.vel_t:
        origin  = i
        for action in robot.actions:
            i = i+1

        break


def redrawScene(win,robot):
    pygame.draw.rect(win,(60,60,70),(0,0,back_width,back_height))#draw whole background. #TODO: replace this with blits
    pygame.draw.rect(win,(70,120,70),(goal.x,goal.y,10,10))
    win.blit(robot.sprite[robot.last_action],(robot.x-26,robot.y-28))
    pygame.display.update()





pygame.init()
back_width, back_height = 1000,1000
win = pygame.display.set_mode((back_width,back_height))
pygame.display.set_caption("Star motion")
clock = pygame.time.Clock()

x = random.randint(0,back_width)
y = random.randint(0,back_height)

vel_t = 5 #translation velocity




goal = PosNode(420,690)
star = Brittlestar(x=x,y=y, o = 0, vel_t = vel_t)




run = True
while run:
    clock.tick(8)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
   
    
    x,y = star.seek(goal)
    redrawScene(win,star)
    
pygame.quit()
