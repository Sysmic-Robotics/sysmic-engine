# Class that holds map geometry data
from world.world import World
from world.entities import Robot
import math

class Map:
    def __init__(self, world : World, discard_point: tuple[float, float]  ):
        # FIELD DATA
        self.x_min = -4.5
        self.x_max = 4.5
        self.y_min = -3
        self.y_max = 3
        self.robots : list[tuple[float, float]] = []
        self.robot_radius = 0.2 # meters
        self.world = world
        self.discard_point = discard_point
        self.create_map()


    def create_map(self):
        self.robots = []
        for robot in self.world.get_robots():
            distance = math.sqrt( (self.discard_point[0] - robot.x)**2 + (self.discard_point[1] - robot.y)**2 )
            if distance > self.robot_radius:
                self.robots.append( (robot.x, robot.y) )
        

    def is_colliding(self, point : tuple[float, float]) -> bool:
        for robot in self.robots:
            distance = math.sqrt( (point[0] - robot[0])**2 + (point[1] - robot[1])**2 )
            if distance <= self.robot_radius:
                return True
        return False