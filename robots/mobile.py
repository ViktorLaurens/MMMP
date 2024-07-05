import os 
import numpy as np
import pybullet as p
from robots.robot import Robot
from utils.pb_conf_utils import wait_for_duration
from utils.planner_utils import Interval

class Env2D: 
    def __init__(self, x_interval: Interval=Interval(-3, 3), y_interval: Interval=Interval(-3, 3), static_obstacles=[]) -> None:
        self.x_interval = x_interval
        self.y_interval = y_interval
        self.static_obstacles = static_obstacles

class MobileRobot(Robot): 
    def __init__(self, urdf_name, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1), scale=1.0, env:Env2D=Env2D()):
        self.env = env
        super().__init__(urdf_name, fixed_base=False, base_position=base_position, base_orientation=base_orientation, scale=scale)
        self.config_space = self.get_config_space()
        self.dimension = len(self.config_space)

    def get_config_space(self):
        return [self.env.x_interval, self.env.y_interval, Interval(-np.pi, np.pi)]

    def set_pose(self, pose):
        if len(pose) == 2: 
            x, y = pose
            position = (x, y, 0)  # Assuming a fixed Z position
            orientation = p.getQuaternionFromEuler((0, 0, 0))
            p.resetBasePositionAndOrientation(self.robot_id, position, orientation) # Directly set the base position and orientation of the robot
        elif len(pose) == 3:
            x, y, theta = pose
            position = (x, y, 0)  # Assuming a fixed Z position
            orientation = p.getQuaternionFromEuler((0, 0, theta))
            p.resetBasePositionAndOrientation(self.robot_id, position, orientation) # Directly set the base position and orientation of the robot
    
    def execute_motion(self, path):
        """
        Execute a planned path consisting of base positions and orientations.
        Each path point is a tuple (x, y, theta), representing 2D position and orientation.
        """
        for pose in path:
            self.set_pose(pose)            
            wait_for_duration(0.1) # Wait a short duration to simulate motion