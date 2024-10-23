from communications.grsim import Grsim
from world.world import World
from world.entities import Robot
import time
from communications.wrapper import CommandSender
import math
from motion.motion import Motion
from world.vision.vision import Vision
import threading

def wait_for_grsim():
    radio : Grsim = Grsim()
    robot : Robot = world.get_robot(1,0)
    while True:
        #radio.communicate_pos_robot(0,0 ,1,0)
        robot = world.get_robot(1,0)
        #radio.communicate_grsim(0,0,5)
        print("Robot pos: ", robot.x, robot.y)
        if robot.x != 0:
            print("Robot pos: ", robot.x, robot.y)
            return
        

if __name__ == '__main__':
    world : World = World(1,1)
    # Init vision
    vision : Vision = Vision("224.5.23.2", 10020, world)
    vision_t = threading.Thread(target= vision.loop)
    vision_t.start()

    time.sleep(1)
    wait_for_grsim()

    motion : Motion = Motion(0, 1, world)
    running = True
    print("Running...")
    last_time = time.time()
    while running:
        # Calculate delta time
        current_time = time.time()
        delta_time = current_time - last_time
        last_time = current_time
        motion.move_linear((2,2))
        