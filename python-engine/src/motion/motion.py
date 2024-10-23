from utility.enums import TaskState
# Path Planning module
from motion.control.control import Control
from world.entities import Robot
from world.world import World

# Control a robot in a desired way
class Motion():
    def __init__(self, robot_id, is_blue, world : World):
        self.id = robot_id
        self.is_blue = is_blue
        self.world = world
        self.current_task = [[0,0], None]
        self.current_control = None
    
    def _is_already_planned(self, task):
        if task[0] != self.current_task[0]:
            return False
        if task[1] != self.current_task[1]:
            return False
        return True
    
    def _set_task():
        pass

    def move_linear(self, point : tuple[float,float]) -> TaskState:
        """
        Move the robot in a linear way.
        This function was built to be called inside a loop.

        Args:
            point (tuple[float, float]): point to move

        Returns:
            TaskState: the state of the task
        """
        robot : Robot = self.world.get_robot(self.is_blue, self.id)
        # Planning
        task = [point, 'move_linear']
        if not self._is_already_planned(task):
            # Then plan a new task
            path = [ point]
            self.current_control = Control(path, self.id, self.is_blue)
            self.current_task = task

        result = self.current_control.control_loop( (robot.x, robot.y) )
        if result:
            return TaskState.SUCCESS
        return TaskState.IN_PROCESS

    def move_free_obstacle(point):
        pass

    def move_around_clockwise(point):
        pass