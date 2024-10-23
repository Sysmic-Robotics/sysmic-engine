from motion.geometric_path.algorithms.map import Map
import math

class FastPlanning:
    def __init__(self, map: Map):
        self.step_subgoal = 0.18
        self.map = map

    def get_path(self, start : tuple[float, float], goal : tuple[float, float]):
        path = [start]
        k = 800

        colliding_point = None
        last_point = start
        sub_goal = start
        for i in range(0, k):
            colliding_point = self.get_point_colliding(sub_goal, goal)
            if colliding_point == None:
                path.append(goal)
                break
            else:
                # Create subgoal
                to_point = colliding_point[0] - last_point[0], colliding_point[1] - last_point[1]
                n_vector = self.normal_vector_up(to_point)
                n_vector = self.normalize_vector2(n_vector)
                sub_goal = colliding_point[0] + n_vector[0]*self.step_subgoal, colliding_point[1] + n_vector[1]*self.step_subgoal
                #path.append(colliding_point)
                if not self.map.is_colliding(sub_goal):
                    last_point = sub_goal
                    path.append(sub_goal)

        return path


    def is_outside_limit(self, point : tuple[float,float]):
        if point[0] > self.map.x_max or point[0] < self.map.x_min:
            return True
        if point[1] > self.map.y_max or point[1] < self.map.y_min:
            return True
        return False


    def get_point_colliding(self, point_a : tuple[float, float], point_b : tuple[float,float]):
        dir_vec = point_b[0] - point_a[0],point_b[1] - point_a[1]
        dir_vec = self.normalize_vector2(dir_vec)
        current_point = point_a
        n_points = int(self.euclidean_distance(point_a, point_b)/self.step_subgoal)
        for i in range(0,n_points):
            if self.map.is_colliding(current_point):
                return current_point
            current_point = current_point[0] + dir_vec[0]*self.step_subgoal , current_point[1] + dir_vec[1]*self.step_subgoal
        return None
        
    
    def normalize_vector2(self, vector: tuple[float, float]) -> tuple[float, float]:
        # Calculate the magnitude (length) of the vector
        magnitude = math.sqrt(vector[0]**2 + vector[1]**2)
        
        # Check if the magnitude is not zero to avoid division by zero
        if magnitude == 0:
            return 0
        
        # Normalize each component of the vector
        normalized_vector = (vector[0] / magnitude, vector[1] / magnitude)
        
        return normalized_vector

    def euclidean_distance(self, point1, point2):
        """
        Calculate the Euclidean distance between two points in a 2D space.
        
        Args:
        point1: A tuple or list representing the coordinates of the first point (x1, y1).
        point2: A tuple or list representing the coordinates of the second point (x2, y2).
        
        Returns:
        The Euclidean distance between point1 and point2.
        """
        distance = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        return distance
    
    def normal_vector_down(self, vector):
        """
        Calculate a normal (perpendicular) vector to the given 2D vector.
        
        Args:
        vector: A tuple or list representing the coordinates of the vector (x, y).
        
        Returns:
        A tuple representing the normal vector.
        """
        x, y = vector
        # Return one of the possible normal vectors
        return (-y, x)
    
    def normal_vector_up(self, vector):
        """
        Calculate a normal (perpendicular) vector to the given 2D vector.
        
        Args:
        vector: A tuple or list representing the coordinates of the vector (x, y).
        
        Returns:
        A tuple representing the normal vector.
        """
        x, y = vector
        # Return the other possible normal vector
        return (y, -x)