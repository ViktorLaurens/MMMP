"""

Pybullet environment for implementation of PRM-based planners

"""
import time
import numpy as np
import pybullet as p

from utils.pb_conf_utils import wait_for_duration, get_view_matrix
from utils.traj_utils import calculate_lspb_trajectory, joint_positions_from_trajectories


# Comes from CBSPRM
class Environment: 
    def __init__(self, agents, obstacles):
        self.agents = agents
        self.robot_models = [agent["model"] for agent in agents]
        self.obstacles = obstacles

        # Initialize robot positions
        for i, agent in enumerate(agents):
            this_model = self.robot_models[i]
            this_model.set_arm_pose(agent["goal"])
        # check for collision
        for i, agent in enumerate(agents):
            this_model = self.robot_models[i]
            if self.robot_collision(this_model):
                raise ValueError(f"Agent {agent['name']} start configuration is in collision.")

    def reset_robots(self):
        for i, agent in enumerate(self.agents):
            this_model = self.robot_models[i]
            this_model.set_arm_pose(agent["start"])
        return
    
    def robot_collision(self, robot):
        if self.robot_self_collision(robot):
            print("Robot self collision")
            return True
        for obstacle in self.obstacles:
            if self.robot_obstacle_collision(robot, obstacle):
                print("Robot obstacle collision")
                return True
        for other_robot in self.robot_models:
            if other_robot != robot and self.robot_robot_collision(robot, other_robot):
                print("Robot robot collision")
                return True
        return False

    def robot_obstacle_collision(self, robot, obstacle, collision_threshold=0.0):
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
        closest_points = p.getClosestPoints(bodyA=robot.r_id, bodyB=obstacle, distance=collision_threshold)
        if closest_points:
            return True
        return False
    
    def robot_robot_collision(self, robot1, robot2, collision_threshold=0.0):
        # Check for collision between the two robots
        closest_points = p.getClosestPoints(bodyA=robot1.r_id, bodyB=robot2.r_id, distance=collision_threshold)
        if closest_points:
            return True
        return 
    
    def robot_self_collision(self, robot, collision_threshold=0.0):
        # check for collision of non-adjacent links
        for i in range(robot.arm_dimension-2):
            for j in range(i+2, robot.arm_dimension):
                closest_points = p.getClosestPoints(bodyA=robot.r_id, linkIndexA=i, bodyB=robot.r_id, linkIndexB=j, distance=collision_threshold)
                if closest_points:
                    return True
        return False
    
    def execute_joint_motion(self, paths):
        # Get the maximum time index across all paths
        times = {r_id: max(path.keys()) for r_id, path in paths.items()}
        max_t = max(times.values())
        time_step = np.round(sorted(paths[list(paths.keys())[0]].keys())[1] - sorted(paths[list(paths.keys())[0]].keys())[0], 1)
        for t in np.arange(0, max_t + time_step, time_step):
            for r_id, path in paths.items():
                r_index = [i for i, agent in enumerate(self.agents) if agent["model"].r_id == r_id][0]
                closest_key = min(path.keys(), key=lambda x: abs(x - t))
                self.robot_models[r_index].set_arm_pose(path[closest_key])
            wait_for_duration(time_step)
        return

    def execute_joint_motion_capturing_frames(self, paths):
        # Get the maximum time index across all paths
        times = {r_id: max(path.keys()) for r_id, path in paths.items()}
        max_t = max(times.values())
        time_step = np.round(sorted(paths[list(paths.keys())[0]].keys())[1] - sorted(paths[list(paths.keys())[0]].keys())[0], 1)
        # np.round(sorted(paths[0].keys())[1] - sorted(paths[0].keys())[0], 1)
        
        frames = []  # List to hold images for the video

        for t in np.arange(0, max_t + time_step, time_step):
            for r_id, path in paths.items():
                r_index = [i for i, agent in enumerate(self.agents) if agent["model"].r_id == r_id][0]
                closest_key = min(path.keys(), key=lambda x: abs(x - t))
                self.robot_models[r_index].set_arm_pose(path[closest_key])
            
            img = self.capture_frame()  # Capture the current state of the simulation
            frames.append(img)
            # wait_for_duration(time_step)  # Pause for the given time step duration
        
        return frames  # Return the captured frames for video creation

    def capture_frame(self, width=640, height=480, camera_point=[0, -1.2, 1.2], target_point=[0, 0, 0]):
        """Capture a single frame from the current PyBullet view."""
        view_matrix = get_view_matrix(camera_point, target_point)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=float(width)/height,
                                                   nearVal=0.1, farVal=100.0)
        _, _, img_arr, _, _ = p.getCameraImage(width, height, view_matrix, proj_matrix,
                                               shadow=1, lightDirection=[1, 1, 1],
                                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
        img = np.reshape(img_arr, (height, width, 4))
        return img[:, :, :3]  # Return RGB, dropping alpha
    
    def execute_interpolated_motion(self, prm, id_paths, t_final):
        # # Get waypoints from the paths
        # waypoints = {r_id: [] for r_id in id_paths.keys()}
        # for r_id, path in id_paths.items():
        #     for node_id in path:
        #         waypoints[r_id].append(np.array(prm.node_id_q_dicts[prm.r_ids.index(r_id)][node_id]))
        
        # calculate interpolated trajectories
        trajectories = {r_id: [] for r_id in id_paths.keys()}
        for r_id in id_paths.keys():
            waypoints = [np.array(prm.node_id_q_dicts[prm.r_ids.index(r_id)][node_id]) for node_id in id_paths[r_id]]
            norms = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
            total_norm = np.sum(norms)
            times = np.cumsum(np.concatenate(([0], norms / total_norm))) * t_final
            for i, t in enumerate(times):
                trajectories[r_id].append((t, waypoints[i]))

        # Simulation parameters
        time_step = 1.0 / 30.0
        p.setTimeStep(time_step)

        # Loop through trajectory points
        t_start = 0.0
        t_stop, _ = trajectories[prm.r_ids[0]][-1]
        for t in np.linspace(t_start, t_stop, num=int((t_stop - t_start) / time_step)):
            for r_id in prm.r_ids:
                joint_positions = self.q_from_interpolated_trajectories(trajectories[r_id], t)   
                if len(joint_positions) != 7:
                    continue                     
                p.setJointMotorControlArray(r_id, range(7), p.POSITION_CONTROL, joint_positions)
                p.stepSimulation()
                time.sleep(time_step)
        return trajectories     
    
    def execute_interpolated_motion_capturing_frames(self, prm, id_paths, t_final):
        # # Get waypoints from the paths
        # waypoints = {r_id: [] for r_id in id_paths.keys()}
        # for r_id, path in id_paths.items():
        #     for node_id in path:
        #         waypoints[r_id].append(np.array(prm.node_id_q_dicts[prm.r_ids.index(r_id)][node_id]))
        
        # calculate interpolated trajectories
        trajectories = {r_id: [] for r_id in id_paths.keys()}
        for r_id in id_paths.keys():
            waypoints = [np.array(prm.node_id_q_dicts[prm.r_ids.index(r_id)][node_id]) for node_id in id_paths[r_id]]
            norms = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
            total_norm = np.sum(norms)
            times = np.cumsum(np.concatenate(([0], norms / total_norm))) * t_final
            for i, t in enumerate(times):
                trajectories[r_id].append((t, waypoints[i]))

        # Simulation parameters
        time_step = 1.0 / 30.0
        p.setTimeStep(time_step)

        # Loop through trajectory points
        t_start = 0.0
        t_stop, _ = trajectories[prm.r_ids[0]][-1]
        frames = []
        for t in np.linspace(t_start, t_stop, num=int((t_stop - t_start) / time_step)):
            for r_id in prm.r_ids:
                joint_positions = self.q_from_interpolated_trajectories(trajectories[r_id], t)   
                if len(joint_positions) != 7:
                    continue                     
                p.setJointMotorControlArray(r_id, range(7), p.POSITION_CONTROL, joint_positions)
                p.stepSimulation()
                time.sleep(time_step)
                img = self.capture_frame()
                frames.append(img)
        return frames, trajectories

    def q_from_interpolated_trajectories(self, trajectories, t):
        joint_positions = []
        for i in range(len(trajectories)-1):
            t1, q1 = trajectories[i]
            t2, q2 = trajectories[i+1]
            if t1 <= t < t2:
                alpha = (t - t1) / (t2 - t1)
                joint_positions = q1 + alpha * (q2 - q1)
                break
            elif t == t2:
                joint_positions = q2
                break
        return joint_positions

    def execute_smooth_motion(self, prm, id_paths, t_final=2.0, effort_factor=0.8):
        # Get waypoints from the paths
        waypoints = {r_id: [] for r_id in id_paths.keys()}
        for r_id, path in id_paths.items():
            for node in path:
                q = np.array(prm.node_id_q_dicts[prm.r_ids.index(r_id)][node])
                waypoints[r_id].append(q)

        # Joint limits for the Panda robot
        joint_limits = {
            'q_max': [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159],
            'q_min': [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
        }
        
        # Velocity and acceleration limits
        velocity_limits = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])  # rad/s
        acceleration_limits = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0])  # rad/s²

        # LSPB trajectories
        trajectories = {r_id: calculate_lspb_trajectory(waypoints[r_id], t_final, effort_factor, joint_limits, velocity_limits, acceleration_limits) for r_id in id_paths.keys()}

        # Simulation parameters
        time_step = 1.0 /30.0
        p.setTimeStep(time_step)

        # Loop through trajectory points
        t_start = 0.0
        t_stop = trajectories[prm.r_ids[0]][0][-1].t2
        for t in np.linspace(t_start, t_stop, num=int((t_stop - t_start) / time_step)):
            for r_id in prm.r_ids:
                joint_positions = joint_positions_from_trajectories(trajectories[r_id], t)
                if len(joint_positions) != 7:
                    continue
                p.setJointMotorControlArray(r_id, range(7), p.POSITION_CONTROL, joint_positions)
                p.stepSimulation()
                time.sleep(time_step)
        return trajectories
    
    def execute_smooth_motion_capturing_frames(self, prm, id_paths, t_final=2.0, effort_factor=0.8):
        # Get waypoints from the paths
        waypoints = {r_id: [] for r_id in id_paths.keys()}
        for r_id, path in id_paths.items():
            for node in path:
                q = np.array(prm.node_id_q_dicts[prm.r_ids.index(r_id)][node])
                waypoints[r_id].append(q)

        # Joint limits for the Panda robot
        joint_limits = {
            'q_max': [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159],
            'q_min': [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
        }
        
        # Velocity and acceleration limits
        velocity_limits = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])  # rad/s
        acceleration_limits = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]) # rad/s²

        # LSPB trajectories
        trajectories = {r_id: calculate_lspb_trajectory(waypoints[r_id], t_final, effort_factor, joint_limits, velocity_limits, acceleration_limits) for r_id in id_paths.keys()}

        # Simulation parameters
        time_step = 1.0 / 30.0
        p.setTimeStep(time_step)

        # Loop through trajectory points
        t_start = 0.0
        t_stop = trajectories[prm.r_ids[0]][0][-1].t2
        frames = []
        for t in np.linspace(t_start, t_stop, num=int((t_stop - t_start) / time_step)):
            for r_id in prm.r_ids:
                joint_positions = joint_positions_from_trajectories(trajectories[r_id], t)
                if len(joint_positions) != 7:
                    continue
                p.setJointMotorControlArray(r_id, range(7), p.POSITION_CONTROL, joint_positions)
                p.stepSimulation()
                time.sleep(time_step)
                img = self.capture_frame()
                frames.append(img)
        return frames, trajectories