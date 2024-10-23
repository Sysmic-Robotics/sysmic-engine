from navigation.map import Map
import math
import random

# Explanation: https://www.youtube.com/watch?v=Oo61tjNJHCk

class RRT:
    def __init__(self, map : Map):
        self.map = map
        self.start = (0.0, 0.0)
        self.goal = (0.0, 0.0)
        # coords, parent
        self.nodes : list[ tuple[tuple[float,float], int]] = []
        self.edges : list[tuple[float,float], tuple[float,float]] = []
        self.max_step : float = 0.09 # in meters

    def get_path(self, start : tuple[float, float], goal : tuple[float, float]):
        # Initialize
        self.start = start
        self.goal = goal
        self.nodes = [ (start, 0) ]
        self.edges = []

        # Compute path
        K : int = 800
        goal_reached : bool = False
        for k in range(0, K):
            random_node = self.get_random_node()
            if k % 10 == 0:
                random_node = self.goal
            goal_reached = self.expand(random_node)
            if(goal_reached):
                break
        if goal_reached:        
            return self.path_to_goal()  
        return []
    

    def path_to_goal(self) -> list[tuple[float, float]]:
        path_indexes = []
        goal_i = len(self.nodes) - 1
        path_indexes.append(goal_i)
        new_pos_i = self.nodes[goal_i][1] # Get parent
        while(new_pos_i != 0): # Finish when reached start pos
            path_indexes.append(new_pos_i)
            new_pos_i = self.nodes[new_pos_i][1]
        path_indexes.append(0)
        # Convert to coords
        path_indexes.reverse()
        path = []
        for i in path_indexes:
            path.append(self.nodes[i][0])
        return path   
    

    def get_random_node(self) -> tuple[float,float]:
        x : float = random.uniform(self.map.x_min, self.map.x_max)
        y : float = random.uniform(self.map.y_min, self.map.y_max)
        return (x,y)
    

    # Return if the goal true if the goal is reached
    def expand(self, random_node : tuple[float, float]):
        if(self.map.is_colliding(random_node)):
            return
        nearest_node_i = self.get_nearest_neighbor_to(random_node)
        nearest_node = self.nodes[nearest_node_i][0]
        new_node : tuple[float, float] = None
        if self.distance(nearest_node, random_node) < self.max_step:
            new_node = nearest_node
        else:
            new_node = self.step(nearest_node, random_node)
        
        if(self.edge_is_colliding(nearest_node, new_node)):
            return    
        self.edges.append( (nearest_node, new_node) ) 
        self.nodes.append( (new_node, nearest_node_i) )

        if self.distance(self.goal, new_node) < self.max_step:
            return True
        return False
    

    def edge_is_colliding(self, node_a : tuple[float, float], node_b : tuple[float, float]):
        return False
        for i in range(0, 101):
            u = i / 100
            x = node_a[0] * u + node_b[0] * (1 - u)
            y = node_a[1] * u + node_b[1] * (1 - u)
            if self.map.is_colliding((x,y)):
                return True
        return False   


    def distance(self, point_a : tuple[float, float], point_b : tuple[float, float]) -> float:
        return math.sqrt((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)
    

    def get_nearest_neighbor_to(self, node):
        min_distance = float('inf')
        nearest_node_i = None
        for i in range( 0, len(self.nodes) ):
            d = self.distance(node, self.nodes[i][0])
            if d < min_distance:
                min_distance = d
                nearest_node_i = i
        return nearest_node_i


    # Create new node between two nodes
    def step(self, from_node : tuple[float, float], to_node : tuple[float, float]): 
        # Angle of vector (to_node - from_node)       
        dir_vec : tuple[float, float] = (to_node[0] - from_node[0],
                               to_node[1] - from_node[1])
        angle = math.atan2(dir_vec[1], dir_vec[0])
        x = from_node[0] + self.max_step * math.cos(angle)
        y = from_node[1] + self.max_step * math.sin(angle)
        return (x, y)   