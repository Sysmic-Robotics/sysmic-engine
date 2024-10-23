from motion.control.bangbang.trajectory_generator import TrajectoryGenerator
from motion.control.bangbang.utils import Constraints
from communications.wrapper import CommandSender
from world.entities import Robot
import time

class Control:
    # Control the robot to follow the desired list of points
    def __init__(self, points_to_follow : list[tuple[float,float]], id: int, is_blue : bool):
        self.points = points_to_follow
        # Robot physics constraints
        self.constraints = Constraints(10,5)
        self.generator = TrajectoryGenerator(self.constraints)
        self.to_next_point = True
        self.trajectory = []

        self.robot = Robot()
        self.robot.id = id
        self.robot.is_blue = is_blue

        self.command_sender = CommandSender()

        self.current_time = time.time()
        self.last_time = 0
        self.delta_time = 0


    def control_loop(self, robot_pos, robot_v = (0,0)) -> bool:
        '''
        Control the robot to follow a list of points
        This function was built to be called inside a loop each 1/60 seconds.

        Args:
            points : list[tuple[float,float]]
            robot : Robot
        
        Returns:
            bool: if there is no point to follow
        '''
        self.current_time = time.time()
        self.delta_time += self.current_time - self.last_time
        self.last_time = self.current_time

        # Ensure to always loop > 1/60 seconds
        if self.delta_time < 1/60:
            
            return False
        self.delta_time = 0
        # ADD TEORICAL POSS
        if self.to_next_point and len(self.points) > 0:
            next_point = self.points.pop(0)
            self.trajectory = self.generator.get_trajectory(robot_v[0], robot_v[1], robot_pos, next_point)
            self.next_point = False
        
        if len(self.trajectory) == 0:
            #print(f"No trajectory found for v: {robot_v} r_pos: {robot_pos} n_point: {next_point}")
            return True
        next_v = self.trajectory.pop(0)

        # Send velocity
        print("vel: ", next_v, "traj len: ", len(self.trajectory))
        self.command_sender.send_robot_data(self.robot.id, 
                                            self.robot.is_blue, 
                                            veltangent=next_v[0], 
                                            velnormal=next_v[1])
        
        robot_v = next_v

        dx_to_brake = ((robot_v[0]**2)/(2*self.constraints.a_max)) 
        dy_to_brake = ((robot_v[1]**2)/(2*self.constraints.a_max))
        if( len(self.points) != 0):
            if (dx_to_brake > abs(robot_pos[0] - next_point[0]) or
            dy_to_brake > abs(robot_pos[1] - next_point[1]) ):
                self.next_point = True
    
        return False