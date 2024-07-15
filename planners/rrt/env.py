"""

Pybullet environment for implementation of RRT-based planners

"""
import numpy as np
import pybullet as p

from utils.pb_conf_utils import wait_for_duration


# Comes from CBSPRM
class Environment: 
    def __init__(self, agents, obstacles):
        self.agents = agents
        self.robot_models = [agent["model"] for agent in agents]
        self.obstacles = obstacles

        # Initialize robot positions
        for i, agent in enumerate(agents):
            this_model = self.robot_models[i]
            this_model.set_arm_pose(agent["start"])
            # check for collisions
            if self.robot_collision(this_model):
                raise ValueError(f"Agent {agent['name']} start configuration is in collision.")
            
    def robot_collision(self, robot):
        for obstacle in self.obstacles:
            if self.robot_obstacle_collision(robot, obstacle):
                print("Robot obstacle collision")
                return True
        for other_robot in self.robot_models:
            if other_robot != robot and self.robot_robot_collision(robot, other_robot):
                print("Robot robot collision")
                return True
        return False

    def robot_obstacle_collision(self, robot, obstacle, collision_threshold=0.01):
        """
        Check for collision between the robot and the obstacle using a specified threshold.

        Parameters:
        - robot: The robot object or identifier.
        - obstacle: The obstacle object or identifier.
        - collision_threshold: The distance below which the robot is considered to be in collision with the obstacle.

        Returns:
        - True if a collision is detected, False otherwise.
        """
        # if obstacle == self.ground:
        #     return False  # Ignore collision with ground
        closest_points = p.getClosestPoints(bodyA=robot.robot_id, bodyB=obstacle, distance=collision_threshold)
        if closest_points:
            return True
        return False
    
    def robot_robot_collision(self, robot1, robot2, collision_threshold=0.01):
        # Check for collision between the two robots
        closest_points = p.getClosestPoints(bodyA=robot1.robot_id, bodyB=robot2.robot_id, distance=collision_threshold)
        if closest_points:
            return True
        return False
    
    def execute_joint_motion(self, paths):
        # Get the maximum time index across all paths
        times = {r_id: max(path.keys()) for r_id, path in paths.items()}
        max_t = max(times.values())
        time_step = np.round(sorted(paths[0].keys())[1] - sorted(paths[0].keys())[0], 1)  
        for t in np.arange(0, max_t + time_step, time_step):
            for r_id, path in paths.items():
                closest_key = min(path.keys(), key=lambda x: abs(x - t))
                self.robot_models[r_id].set_arm_pose(path[closest_key])
            # wait_for_duration(time_step)

    def execute_joint_motion_capturing_frames(self, paths):
        # Get the maximum time index across all paths
        times = {r_id: max(path.keys()) for r_id, path in paths.items()}
        max_t = max(times.values())
        time_step = np.round(sorted(paths[0].keys())[1] - sorted(paths[0].keys())[0], 1)
        
        frames = []  # List to hold images for the video

        for t in np.arange(0, max_t + time_step, time_step):
            for r_id, path in paths.items():
                closest_key = min(path.keys(), key=lambda x: abs(x - t))
                self.robot_models[r_id].set_arm_pose(path[closest_key])
            
            img = self.capture_frame()  # Capture the current state of the simulation
            frames.append(img)
            # wait_for_duration(time_step)  # Pause for the given time step duration
        
        return frames  # Return the captured frames for video creation

    def capture_frame(self, width=640, height=480):
        """Capture a single frame from the current PyBullet view."""
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0, 0, 1], distance=5,
                                                          yaw=30, pitch=-30, roll=0, upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=float(width)/height,
                                                   nearVal=0.1, farVal=100.0)
        _, _, img_arr, _, _ = p.getCameraImage(width, height, view_matrix, proj_matrix,
                                               shadow=1, lightDirection=[1, 1, 1],
                                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
        img = np.reshape(img_arr, (height, width, 4))
        return img[:, :, :3]  # Return RGB, dropping alpha