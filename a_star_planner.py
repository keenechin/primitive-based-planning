import numpy as np
import pygame as pg
import heapq
import time

class A_star:
    """ 
    Class for A* planning
    """

    class Node:
        """
        A* Node
        """

        def __init__(self, x, y, goal = None):
            self.x = x
            self.y = y
            self.parent = None
            self.action_into = 0
            self.children = []
            if goal is None:
                goal_dist = np.Inf
            else:
                goal_dist = int(np.linalg.norm([goal.x-self.x, goal.y-self.y]))
            self.goal_dist = goal_dist
            self.cost_so_far = 0
    
    class KeyHeap(object):
        def __init__(self, initial=None, key=lambda x:x):
            self.key = key
            if initial:
                self._data = [(key(item), num, item) for num, item in enumerate(initial)]
                heapq.heapify(self._data)
            else:
                self._data = []
            self.counter = len(self._data)

        def push(self, item):
            heapq.heappush(self._data, (self.key(item), self.counter, item))
            self.counter +=1

        def pop(self):
            return heapq.heappop(self._data)[2]
    
    def __init__(self, robot, environment, max_iter=500):
        goal = environment.goal
        self.end = self.Node(goal[0], goal[1], goal = None)
        self.start = self.Node(robot.x, robot.y, goal = self.end)
        self.max_iter = max_iter
        self.obstacle_list = environment.obstacles
        self.node_list = self.KeyHeap(key = lambda node: node.goal_dist )   #maybe make key a function instead of class attribute 
        self.agent = robot 
        self.goal_tol = self.agent.vel_t
        self.env = environment


    def collides(self,x,y,obstacle_list):

        for obst in obstacle_list:
            dist = np.linalg.norm([x-obst[0],y - obst[1]])

            rad = obst[2]
            if dist < rad:
                return True
        return False

    def expand(self,node):
        children = []
        for action in self.agent.actions:
            x,y = self.agent.peek(action,x=node.x, y = node.y)
            if self.collides(x,y,self.obstacle_list):
                print("Collision!")
                continue

            def is_duplicate(x,y):
                min_dist = np.Inf
                duplicate = False
                for datum in self.node_list._data:
                    dnode = datum[2]
                    dist = np.linalg.norm([x-dnode.x, y - dnode.y])
                    if dist < min_dist:
                        min_dist = dist
                    if abs(x- dnode.x) < 0.8*self.agent.vel_t and abs(y - dnode.y) < 0.8*self.agent.vel_t:
                        #print(["is duplicate: ", x,y])
                        duplicate = True
                        break
                #print({"Min dist": min_dist})
                return duplicate

            if is_duplicate(x,y):
                continue 
            
            new_node = self.Node(x,y, goal = self.end)

           # print((x,y,self.end.x,self.end.y,goal_dist+node.cost_so_far+self.agent.vel_t))

            if  new_node.goal_dist >= self.goal_tol:
                new_node.parent = node
                new_node.action_into = action
                new_node.cost_so_far = new_node.parent.cost_so_far + self.agent.vel_t
                children.append(new_node)
            else:
                self.end.parent = node
                self.end.action_into = action
                children.append(self.end)  
                break
        node.children = children
        return children


    def create_graph(self, animation = True):
        expanded = set([])
        self.node_list.push(self.start)
        for _ in range(self.max_iter):
            time.sleep(0.25)
            best_node = self.node_list.pop()
            self.env.update(self.node_list)

            tmp_nodes = []
            while best_node in expanded:
                tmp_nodes.append(best_node)
                best_node = self.node_list.pop()
            for tnode in tmp_nodes:
                self.node_list.push(tnode)
            children = self.expand(best_node)
            expanded.add(best_node)
            self.node_list.push(best_node) #something about popping breaks it
      

            for node in children:
                self.node_list.push(node)
                if node is self.end:
                    print ("Found end")
                    self.env.update(self.node_list)
                    return self.node_list
            print(len(self.node_list._data))
        return None


    def plan(self):
        graph = self.create_graph()
        """
        for nd in graph._data:
            node = nd[2]
            print((node.x,node.y))
        """
        path = []
        path_nodes = self.KeyHeap(key = lambda n: n.goal_dist + n.cost_so_far)
        if graph is None:
            print("Unable to reach goal.")
        else:
            node = graph.pop()
            while node is not None:
                path.append(node.action_into)
                path_nodes.push(node)
                node = node.parent

        path.reverse()
        return path, path_nodes



            

if __name__ == "__main__":
    from robot import Brittlestar
    from environment import Environment
    import pygame
    import random
    pygame.init()

    obst1 = [(300,300,50), (200,500,100),(400,500,100),(600,500,100)]
    obst2 = [(200,500,80),(300,500,80),(400,500,80),(500,500,80),
    (600,500,80),(700,500,80),(800,500,80),(900,500,80),(1000,500,80)]
    env  =  Environment(start =  (random.randint(10,1000),random.randint(10,100)), goal = (420,690), obstacles= obst2)
    robot = Brittlestar(x=env.start[0], y = env.start[1], o = 0,  vel_t = 50)
    planner = A_star(robot,env,max_iter=10000)
    path,path_nodes = planner.plan()
    print(path)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
        env.update(path_nodes)


