import pygame

class Environment():

    def __init__(self,start,goal,obstacles):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.back_width, self.back_height = 1000,1000
        self.win = pygame.display.set_mode((self.back_width,self.back_height))
        pygame.display.set_caption("Star motion")
        self.clock = pygame.time.Clock()
        self.graph = None
        self.color = [(255,0,0), (204,255,0),(0,255,102),(204,0,255),(0,102,255)]




    def update(self,graph):
        self.graph = graph
        pygame.draw.rect(self.win,(60,60,70),(0,0,self.back_width,self.back_height))#draw whole background. #TODO: replace this with blits
        pygame.draw.rect(self.win,(70,120,70),(self.goal[0],self.goal[1],10,10))
        for element in graph._data:
            node = element[2]
            pygame.draw.circle(self.win, self.color[node.action_into], (node.x,node.y), 4)
        for obst in self.obstacles:
            pygame.draw.circle(self.win, (122,0,0), (obst[0],obst[1]),obst[2])
        pygame.display.update()