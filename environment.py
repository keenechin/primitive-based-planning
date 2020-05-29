
class Environment():

    def __init__(self,start,goal,obstacles):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.back_width, self.back_height = 1000,1000
        self.graph = None




    def update(self,graph):
        self.graph = graph
        for element in graph._data:
            node = element[2]