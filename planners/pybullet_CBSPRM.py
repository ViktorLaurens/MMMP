"""

Python implementation of Conflict-based search for pybullet implementation

"""
from itertools import combinations, product
from copy import deepcopy
import math
import pprint
import time

from matplotlib import pyplot as plt
import numpy as np
import imageio
import pybullet as p
from scipy.spatial import KDTree

# Define the path to your project root directory
project_root = 'C:\\Users\\vikto\\MMMP'

# Add the project root directory to sys.path if it's not already included
import sys
import os
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from utils.planner_utils import Interval
from robots.panda import Panda
from utils.pb_conf_utils import add_data_path, connect, disconnect, pause_sim, set_camera_pose, wait_for_duration
from utils.ik_utils import calculate_arm_ik
 # parameter
# N_SAMPLES = 100  # number of samples
# N_KNN = 5  # number of edge from one sampled point
# MAX_EDGE_LEN = 2  # [m] Maximum edge length
# TYPE = 'nn' # 'distance' or 'nn'
# LOCAL_STEP = 0.05  # [rad] Step size for local planner

# constants
# INF = float("inf")

def main():
    connect(use_gui=True)
    add_data_path()
    set_camera_pose(camera_point=[0, -1.2, 1.2])
    ground = p.loadURDF("plane.urdf")

    robot1 = Panda(fixed_base=True, base_position=(0.6, 0, 0.02), base_orientation=(0, 0, 1, 0))  # Specify base position and orientation for robot1
    robot2 = Panda(fixed_base=True, base_position=(-0.6, 0, 0.02), base_orientation=(0, 0, 0, 1))  # Specify base position and orientation for robot2

    # Disable collision between the robot and the ground plane
    p.setCollisionFilterPair(robot1.robot_id, ground, -1, -1, enableCollision=0)
    p.setCollisionFilterPair(robot2.robot_id, ground, -1, -1, enableCollision=0)

    tool_position_goal1 = (-0.3, 0.4, 0.5)
    tool_orientation_goal1 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    tool_pose_goal1 = (tool_position_goal1, tool_orientation_goal1)
    arm1_goal = calculate_arm_ik(robot1.robot_id, robot1.tool_link, tool_pose_goal1)
    arm1_goal = tuple(round(c, 3) for c in arm1_goal)
    arm1_goal = (-0.319, 1.221, -0.255, -0.199, 0.244, 1.456, -2.724)
    print(arm1_goal)

    if arm1_goal is not None:
        pause_sim('Show goal pose for arm 1?')
        # print("IK Configuration for panda 1 goal pose:", arm1_goal)
        robot1.set_pose(arm1_goal)
    else:
        print("No IK solution found for panda 1 goal pose.")

    # Calc and set start of arm 1
    # pause_sim('Calculate IK for start of arm 1?')
    tool_position_start1 = (-0.3, -0.4, 0.5)
    tool_orientation_start1 = p.getQuaternionFromEuler([np.radians(-180), 0, 0])
    tool_pose_start1 = (tool_position_start1, tool_orientation_start1)
    arm1_start = calculate_arm_ik(robot1.robot_id, robot1.tool_link, tool_pose_start1)
    arm1_start = tuple(round(c, 3) for c in arm1_start)
    arm1_start = (0.41, 1.103, -0.243, -0.419, 0.219, 1.558, -2.045)
    print(arm1_start)

    if arm1_start is not None:
        pause_sim('Show start pose for arm 1?')
        # print("IK Configuration for panda 1 start pose:", arm1_start)
        robot1.set_pose(arm1_start)
    else:
        print("No IK solution found for panda 2 start pose.")

    # Calc and set goal of arm 2
    # pause_sim('Calculate IK for goal of arm 2')
    tool_position_goal2 = (0.1, -0.4, 0.5)
    tool_orientation_goal2 = p.getQuaternionFromEuler([np.radians(-180), 0, 0])
    tool_pose_goal2 = (tool_position_goal2, tool_orientation_goal2)
    arm2_goal = calculate_arm_ik(robot2.robot_id, robot2.tool_link, tool_pose_goal2)
    arm2_goal = tuple(round(c, 3) for c in arm2_goal)
    arm2_goal = (-0.416, 1.116, -0.012, -0.293, 0.011, 1.434, 0.366)
    print(arm2_goal)

    if arm2_goal is not None:
        pause_sim('Show goal pose for arm2?')
        # print("IK Configuration for panda 2 goal pose:", arm2_goal)
        robot2.set_pose(arm2_goal)
    else:
        print("No IK solution found for panda 2 goal pose.")

    # Calc and set start of arm 2
    # pause_sim('Calculate IK for start of arm 2')
    tool_position_start2 = (0.1, 0.4, 0.5)
    tool_orientation_start2 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    tool_pose_start2 = (tool_position_start2, tool_orientation_start2)
    arm2_start = calculate_arm_ik(robot2.robot_id, robot2.tool_link, tool_pose_start2)
    arm2_start = tuple(round(c, 3) for c in arm2_start)
    arm2_start = (0.413, 1.057, 0.02, -0.4, -0.018, 1.484, 1.207)
    print(arm2_start)

    if arm2_start is not None:
        pause_sim('Show start pose for arm2?')
        # print("IK Configuration for panda 2 start pose:", arm2_start)
        robot2.set_pose(arm2_start)
    else:
        print("No IK solution found for panda 2 start pose.")

    config_space1 = create_intervals_from_tuples(arm1_start, arm1_goal)
    print(config_space1)
    config_space2 = create_intervals_from_tuples(arm2_start, arm2_goal)
    print(config_space2)

    # agents = [
    #         {"start": arm1_start, "goal": arm1_goal, "name": "agent0", "model": robot1, "roadmap": roadmap1}, 
    #         {"start": arm2_start, "goal": arm2_goal, "name": "agent1", "model": robot2, "roadmap": roadmap2}
    #     ]

    agents = [
            {"name": "agent1", "start": arm1_start, "goal": arm1_goal, "model": robot1, "roadmap": None}, 
            {"name": "agent2", "start": arm2_start, "goal": arm2_goal, "model": robot2, "roadmap": None}
        ]
    
    obstacles = [ground]

    env = Environment(agents, obstacles)

    pause_sim('Learn?')
    start_time = time.time()
    prm = CBSPRM(env, maxdist=10, k1=10, k2=5, build_type='kdtree', n=1000, t=10, time_step=0.1, local_step=0.05)
    learn_duration = time.time()-start_time
    print(f"Learning duration: {learn_duration}")
    # print(f"Edges: {prm.edge_dicts}")

    robot1.set_pose(arm1_start)
    robot2.set_pose(arm2_start)

    pause_sim('Query?')
    start_time = time.time()
    paths = prm.query()
    query_duration = time.time()-start_time
    if not paths:
        print("Solution not found")
        pause_sim('Disconnect?')
        disconnect()
        return
    print(f"Solution: {paths}")
    print(f"Query duration: {query_duration}")

    pause_sim('execute joint motion?')
    frames = env.execute_joint_motion_capturing_frames(paths)
    pause_sim('Disconnect?')
    disconnect()

    flag = input('Save video? (y/N): ')
    if flag.lower() == 'y':
        # Define where to save the video
        now = time.strftime("%Y%m%d_%H%M%S")
        directory = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Videos'
        filename = f'\simulation_{len(agents)}pandas_{robot1.base_position}_{now}.mp4'

        # Save the captured frames as a video
        imageio.mimsave(directory + filename, frames, fps=30)  # Define the FPS as needed
        print(f"Video saved as {directory + filename}")
    else: 
        print("Video not saved.")
    
# def execute_joint_motion(robots, paths):
#     max_t = max(len(path) for path in paths)        # Find the maximum timestep required for any robot to complete its path
#     for t in range(max_t):                          # Loop through each timestep
#         for i, robot in enumerate(robots):          # Update each robot's position for the current timestep
#             if t < len(paths[i]):                   # Check if the current robot's path has a position for timestep 't'
#                 robot.set_pose(paths[i][t])         # Set the robot's pose to the position at timestep 't'
#             else:
#                 # Optional: Handle cases where the robot has already reached its goal
#                 # e.g., keep the robot stationary at its last position
#                 pass
        
#         # Pause for a short duration to simulate motion
#         wait_for_duration(0.15)

def create_intervals_from_tuples(tuple1, tuple2):
    return [Interval(lower=min(a, b), upper=max(a, b)) for a, b in zip(tuple1, tuple2)]

class Node:
    def __init__(self, id, q):
        self.id = id
        self.q = tuple([round(c, 2) for c in q])
    def __eq__(self, other):
        return self.id == other.id
    def __hash__(self):
        return hash(str(self.id) + str(self.q))
    def __str__(self):
        return str(self.id, self.q)   
    
class edge:
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2
    def __eq__(self, other):
        return self.node1 == other.node1 and self.node2 == other.node2
    def __hash__(self):
        return hash(str(self.node1) + str(self.node2))
    def __str__(self):
        return str((self.node1, self.node2))
    
class State:
    def __init__(self, t, node):
        self.t = t
        # self.node = node
        # self.id = node.id
        self.q = node.q
    def __eq__(self, other):
        return self.t == other.t and self.q == other.q
    def __hash__(self):
        return hash(str(self.t) + str(self.q))
    def is_equal_except_time(self, state):
        return self.q == state.q
    def __str__(self):
        return str((self.t, self.q))

class Environment: 
    def __init__(self, agents, obstacles):
        self.agents = agents
        self.robot_models = [agent["model"] for agent in agents]
        self.obstacles = obstacles

        # Initialize robot positions
        for i, agent in enumerate(agents):
            this_model = self.robot_models[i]
            this_model.set_pose(agent["start"])
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
                self.robot_models[r_id].set_pose(path[closest_key])
            wait_for_duration(time_step)

    def execute_joint_motion_capturing_frames(self, paths):
        # Get the maximum time index across all paths
        times = {r_id: max(path.keys()) for r_id, path in paths.items()}
        max_t = max(times.values())
        time_step = np.round(sorted(paths[0].keys())[1] - sorted(paths[0].keys())[0], 1)
        
        frames = []  # List to hold images for the video

        for t in np.arange(0, max_t + time_step, time_step):
            for r_id, path in paths.items():
                closest_key = min(path.keys(), key=lambda x: abs(x - t))
                self.robot_models[r_id].set_pose(path[closest_key])
            
            img = self.capture_frame()  # Capture the current state of the simulation
            frames.append(img)
            wait_for_duration(time_step)  # Pause for the given time step duration
        
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

class Conflict:
    def __init__(self):
        self.t = -1
        self.r_id1 = -1
        self.r_id2 = -1
        self.q1 = tuple()
        self.q2 = tuple()
    def __str__(self):
        return '(' + str(self.t) + ', ' + str(self.r_id1) + ', ' + str(self.r_id2) + \
             ', '+ str(self.q1) + ', ' + str(self.q2) + ')'
    
class Constraint:
    def __init__(self, r_1_id, r_2_id, q2, t):
        self.r_1_id = r_1_id    # robot 1 id (robot which movement is to be constrained)
        self.r_2_id = r_2_id    # robot 2 id (robot which robot 1 is in conflict with)
        self.q2 = q2            # (configuration of robot 2 at time of conflict)
        self.t = t              # time of conflict
    # def __eq__(self, other):
    #     return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.t)+str(self.q2))
    def __str__(self):
        return '(' + str(self.t) + ', '+ str(self.q2) + ')'

class HighLevelNode:
    def __init__(self):
        self.solution = {}          # keys: agents, values: paths (= lists of states)
        self.discretized_solution = {}
        self.constraint_dict = {}   # keys: agents, values: constraints for agent
        self.cost = 0
    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost
    def __hash__(self):
        return hash((self.cost))
    def __lt__(self, other):
        return self.cost < other.cost

class CBSPRM:
    def __init__(self, environment, maxdist, k1=20, k2=10, build_type='kdtree', n=100, t=10, time_step=0.1, local_step=0.05) -> None:
        self.robot_ids = np.linspace(0, len(environment.robot_models)-1, len(environment.robot_models), dtype=int)
        self.config_spaces = self.create_config_spaces(environment.robot_models)
        self.maxdist = maxdist
        self.k1 = k1
        self.k2 = k2
        self.build_type = build_type
        self.n = n
        self.t = t
        self.node_lists = self.create_node_lists(environment.robot_models)
        self.edge_dicts = self.create_edge_dicts(environment.robot_models)
        self.node_id_q_dicts = self.create_node_id_q_dict(environment.robot_models)
        self.node_q_id_dicts = self.create_node_q_id_dict(environment.robot_models)   

        # collision checking
        self.time_step = time_step
        self.local_step = local_step

        # unpack environmental data
        self.env = environment
        self.agents = environment.agents                
        self.robot_models = environment.robot_models    
        self.obstacles = environment.obstacles

        # call method to generate roadmap
        self.generate_roadmaps()

    #  Initialization methods
    def create_config_spaces(self, robot_models):
        config_spaces = []
        for model in robot_models:
            config_spaces.append(model.config_space)
        return config_spaces
    
    def create_node_lists(self, robot_models):
        node_list = []
        for _ in robot_models:
            node_list.append([])
        return node_list
    
    def create_edge_dicts(self, robot_models):
        edge_list = []
        for _ in robot_models:
            edge_list.append([])
        return edge_list
    
    def create_node_id_q_dict(self, robot_models):
        node_id_q_dict = []
        for _ in robot_models:
            node_id_q_dict.append({})
        return node_id_q_dict
    
    def create_node_q_id_dict(self, robot_models):
        node_q_id_dict = []
        for _ in robot_models:
            node_q_id_dict.append({})
        return node_q_id_dict
    
    def generate_node_id_q_dict(self, nodes):
        return {node.id: node.q for node in nodes}
    
    def generate_node_q_id_dict(self, nodes):
        return {tuple(node.q): node.id for node in nodes}
    
    # learning methods
    def generate_roadmaps(self):
        for r_id in self.robot_ids:
            self.generate_roadmap(r_id)
        return

    def generate_roadmap(self, r_id):
        # Build roadmap based on time
        if self.build_type == 't':
            start_time = time.time()
            while time.time() - start_time < self.t:
                sample = np.random.uniform([interval.lower for interval in self.config_spaces[r_id]], 
                                            [interval.upper for interval in self.config_spaces[r_id]], 
                                            len(self.config_spaces[r_id])
                                            )
                sample = np.round(sample, 2)
                new_node = Node(len(self.node_lists[r_id]), sample)
                self.node_lists[r_id].append(new_node)
                self.edge_dicts[r_id].update({new_node.id: []})
                self._find_and_connect_neighbors(new_node)
            self.node_id_q_dicts[r_id] = self.generate_node_id_q_dict(self.node_lists[r_id])
            self.node_q_id_dicts[r_id] = self.generate_node_q_id_dict(self.node_lists[r_id])
            return
        # build roadmap based on number of nodes
        elif self.build_type == 'n':
            for i in range(self.n):
                sample = np.random.uniform([interval.lower for interval in self.config_spaces[r_id]], 
                                            [interval.upper for interval in self.config_spaces[r_id]], 
                                            len(self.config_spaces[r_id])
                                            )
                sample = np.round(sample, 2)
                new_node = Node(i, sample)
                self.node_lists[r_id].append(new_node)
                self.edge_dicts[r_id].update({new_node.id: []})
                self._find_and_connect_neighbors(r_id, new_node)
            self.node_id_q_dicts[r_id] = self.generate_node_id_q_dict(self.node_lists[r_id])
            self.node_q_id_dicts[r_id] = self.generate_node_q_id_dict(self.node_lists[r_id])
            return
        # build roadmap based on kdtree, nodes are set a priori, no update of tree possible once created
        elif self.build_type == 'kdtree':
            # sample nodes
            samples = self.sample_valid(self.n, r_id)
            # samples = np.random.uniform([interval.lower for interval in self.config_spaces[r_id]],
            #                             [interval.upper for interval in self.config_spaces[r_id]],
            #                             (self.n, len(self.config_spaces[r_id]))
            #                             )
            # samples = np.round(samples, 2)
            self.node_lists[r_id] = [Node(i, sample) for i, sample in enumerate(samples)]
            self.node_id_q_dicts[r_id] = self.generate_node_id_q_dict(self.node_lists[r_id])
            self.node_q_id_dicts[r_id] = self.generate_node_q_id_dict(self.node_lists[r_id])

            # create roadmap
            self.edge_dicts[r_id] = {node.id: [] for node in self.node_lists[r_id]}
            tree = KDTree(samples)  
            for i, node in enumerate(self.node_lists[r_id]):
                id = node.id
                q = node.q
                # neighbors = tree.query_ball_point(q, r=self.maxdist)
                distances, neighbor_idxs = tree.query(q, k=self.k1+1) # +1 to exclude the node itself
                for d, id2 in zip(distances, neighbor_idxs):
                    if d > 0 \
                    and d < self.maxdist \
                    and len(self.edge_dicts[r_id][id]) < self.k2 \
                    and self.is_collision_free_edge(q, self.node_id_q_dicts[r_id][id2], r_id):
                        self.edge_dicts[r_id][id].append(id2)
            return
        # raise error if build_type is not 't' or 'n' or 'kdtree'
        else:
            raise ValueError(f"Unexpected build_type {self.build_type}. Expected 't' or 'n' or 'kdtree'.")

    def sample_valid(self, n, r_id):
        samples = []
        while len(samples) < n: 
            sample = np.random.uniform([interval.lower for interval in self.config_spaces[r_id]], 
                                        [interval.upper for interval in self.config_spaces[r_id]], 
                                        len(self.config_spaces[r_id])
                                        )
            sample = np.round(sample, 2)
            if self.is_collision_free_sample(sample, r_id):
                samples.append(sample)
        return samples

    def _find_and_connect_neighbors(self, r_id, new_node):
        candidate_neighbors = []
        distances = [(self.distance(new_node.config, node.config), node) 
                     for node in self.node_lists[r_id][:-1]]
        distances.sort()  # Sort by distance
        
        for distance, neighbor in distances:
            if distance > self.maxdist or len(candidate_neighbors) == self.k1:
                break
            if not self._same_component(new_node, neighbor):
                candidate_neighbors.append(neighbor)
                
        for neighbor in candidate_neighbors:
            self._try_connect(r_id, new_node, neighbor)

    def _same_component(self, node1, node2):
        return False
    
    def _try_connect(self, r_id, node1, node2):
        if self.is_collision_free_edge(node1, node2):
            self.edge_dicts[r_id][node1.id].append(node2.id)
            self.edge_dicts[r_id][node2.id].append(node1.id)
    
    #  Distance metric
    def distance(self, q1, q2):
        # Placeholder for distance calculation between two configurations
        return np.linalg.norm(np.array(q1) - np.array(q2))

    #  Collision checking
    def is_collision_free_sample(self, sample, robot_id):
        # Check for collisions between the robot and the obstacles
        self.robot_models[robot_id].set_pose(sample)
        for obstacle in self.obstacles:
            if self.env.robot_obstacle_collision(self.robot_models[robot_id], obstacle):
                return False
        return True
    
    def is_collision_free_node(self, node, robot_id):
        return self.is_collision_free_sample(node.q, robot_id)

    def is_collision_free_edge(self, q1, q2, robot_id):
        # Check for collisions along the line segment between s and g
        q1 = np.array(q1)
        q2 = np.array(q2)
        for t in np.arange(0, 1, self.local_step):
            sample = q1 + t * (q2 - q1)
            if not self.is_collision_free_sample(sample, robot_id):
                return False
        return True
    
    def robot_robot_collision(self, q1, q2, r_1_id, r_2_id):
        self.robot_models[r_1_id].set_pose(q1)
        self.robot_models[r_2_id].set_pose(q2)
        return self.env.robot_robot_collision(self.robot_models[r_1_id], self.robot_models[r_2_id])

    def transition_valid(self, r_id, d_to_n_1, n_1_id, n_2_id, conflict_times, constraints_from_t):
        # Get the configurations for the current node and the neighbor node
        q1 = np.array(self.node_id_q_dicts[r_id][n_1_id])
        q2 = np.array(self.node_id_q_dicts[r_id][n_2_id])

        # Calculate the time interval between the current node and the neighbor node
        t1 = float(d_to_n_1 / self.local_step) * self.time_step
        t2 = t1 + np.linalg.norm(q2 - q1) / self.local_step * self.time_step
        delta_t = t2 - t1

        # Append the start time to the current and neighbor configurations
        q1_t1 = np.append(q1, t1)
        q2_t2 = np.append(q2, t2)

        # Calculate the start and end times for the interval
        interval_t1 = np.round(np.ceil(t1 / self.time_step) * self.time_step, 1)
        interval_t2 = np.round(np.floor(t2 / self.time_step) * self.time_step, 1)

        # Discretize the path from the start configuration to the end configuration
        discretized_path = {}
        for t in np.round(np.arange(interval_t1, interval_t2 + self.time_step, self.time_step), 1):   # np_arange(start, stop) -> [start, stop) 
            q_at_t = self.find_q_in_q_t_interval_given_t(q1_t1, q2_t2, t)
            discretized_path.update({np.round(t, 1): q_at_t})
            if t in conflict_times: 
                constraints = constraints_from_t[t]
                for constraint in constraints:
                    r_2_q = constraint.q2
                    if self.robot_robot_collision(q_at_t, r_2_q, r_id, constraint.r_2_id):
                        return False   
        return True
    
    def find_q_in_q_t_interval_given_t(self, q1_t1, q2_t2, t):
        delta_t = q2_t2[-1] - q1_t1[-1]
        x = (t - q1_t1[-1]) / delta_t
        return np.round(q1_t1[:-1] + (q2_t2[:-1] - q1_t1[:-1]) * x, 2)

    #  Query methods
    def query(self):
        start = HighLevelNode()
        # self.n_ct_nodes += 1
        for r_id in self.robot_ids:
            self.add_start_goal_nodes(r_id)
            start.constraint_dict[r_id] = []
        start.solution = self.compute_solution(start.constraint_dict)
        start.discretized_solution = {r_id: self.discretize_path_in_time(r_id, start.solution[r_id]) for r_id in start.solution}
        if not start.solution:
            return {}
        start.cost = self.compute_cost(start.solution)
        
        open_set = {start}
        closed_set = set()

        while open_set: 
            P = min(open_set)
            open_set.remove(P)
            closed_set.add(P)

            first_conflict = self.get_first_conflict_in_time(P.discretized_solution)
            
            if first_conflict is None:
                print("Solution found")
                return P.discretized_solution
            
            constraints_from_conflict = self.get_constraints_from_conflict(first_conflict)

            for r_id in first_conflict.r_id1, first_conflict.r_id2:
                new_node = deepcopy(P)
                # self.n_ct_nodes += 1
                new_node.constraint_dict[r_id].append(constraints_from_conflict[r_id])
                new_node.solution = self.compute_solution(new_node.constraint_dict)
                new_node.discretized_solution = {r_id: self.discretize_path_in_time(r_id, new_node.solution[r_id]) for r_id in new_node.solution}
                if not new_node.solution:
                    continue
                new_node.cost = self.compute_cost(new_node.solution)

                if new_node not in closed_set:
                    open_set |= {new_node}
        return {}

    def compute_solution(self, constraint_dict):
        solution = {}
        for r_id in self.robot_ids:
            constraints = constraint_dict[r_id]
            id_path = self.query_robot(r_id, constraints) # path is id_path
            if not id_path:
                return {}
            solution.update({r_id: id_path})
        return solution
    
    def compute_cost(self, solution={}):
        total_cost = 0
        for r_id, id_path in solution.items():
            cost = 0
            for i in range(len(id_path) - 1):
                cost += self.distance(self.node_id_q_dicts[r_id][id_path[i]], self.node_id_q_dicts[r_id][id_path[i+1]])
            total_cost += cost
        return total_cost

    def get_first_conflict_in_time(self, d_solution):
        conflict = Conflict()
        # for r_id, id_path in solution.items():
        #     discretized_solution[r_id] = self.discretize_path_in_time(r_id, id_path)
        max_t = max([max(t_q_path.keys()) for t_q_path in d_solution.values()])
        for t in np.round(np.arange(0, max_t, self.time_step), 1):
            for r_id1, r_id2 in combinations(d_solution.keys(), 2): 
                t_q_path1 = d_solution[r_id1]
                t_q_path2 = d_solution[r_id2]
                q1 = t_q_path1[min(t_q_path1.keys(), key=lambda x: abs(x - t))]  # this makes sure that for max(time_keys) in t_q_path1 < t the goal config is selected
                q2 = t_q_path2[min(t_q_path2.keys(), key=lambda x: abs(x - t))]  # this makes sure that for max(time_keys) in t_q_path2 < t the goal config is selected
                if self.robot_robot_collision(q1, q2, r_id1, r_id2):
                    conflict.t = t
                    conflict.r_id1 = r_id1
                    conflict.r_id2 = r_id2
                    conflict.q1 = q1
                    conflict.q2 = q2
                    return conflict
        return None
    
    def get_constraints_from_conflict(self, conflict):
        constraints = {}
        t = conflict.t
        r_id1 = conflict.r_id1
        r_id2 = conflict.r_id2
        q1 = conflict.q1
        q2 = conflict.q2
        constraints[r_id1] = Constraint(r_id1, r_id2, q2, t)
        constraints[r_id2] = Constraint(r_id2, r_id1, q1, t)
        return constraints

    def query_robot(self, r_id, constraints={}): 
        path = []
        # Use A* to find a path between the nearest start and goal nodes
        a_star_start_time = time.perf_counter()
        a_path, a_distance = self.a_star_search(r_id, constraints)
        a_star_duration = time.perf_counter() - a_star_start_time
        print(f"A*: Composite path in {a_star_duration:.6f} seconds with distance {a_distance:.2f}")

        # Delete start and goal nodes from the roadmap
        # self.delete_start_goal_nodes(r_id, start_config, goal_config, start_node, goal_node) # Reset roadmap

        # Store the path for the agent
        path = a_path
        if not path:
            print(f"No path found for robot {r_id}.")
            return {}
        else: 
            # path = self.discretize_path(r_id, path)
            return path
        
    def add_start_goal_nodes(self, r_id):
        start_config = self.agents[r_id]['start']
        goal_config = self.agents[r_id]['goal']

        if not self.is_collision_free_sample(start_config, r_id): 
            print(f"Start configuration for robot {r_id} is in collision.")
            return
        if not self.is_collision_free_sample(goal_config, r_id):
            print(f"Goal configuration for robot {r_id} is in collision.")
            return
        
        # Add start node to roadmap based on distance 
        self.edge_dicts[r_id].update({-1: []})  # -1 -> start config
        for node in self.node_lists[r_id]:
            if len(self.edge_dicts[r_id][-1]) == self.k2:
                break
            if self.distance(start_config, node.q) <= self.maxdist and self.is_collision_free_edge(start_config, node.q, r_id):
                self.edge_dicts[r_id][-1].append(node.id)
                self.edge_dicts[r_id][node.id].append(-1)
        self.node_id_q_dicts[r_id].update({-1: tuple(start_config)})
        self.node_q_id_dicts[r_id].update({tuple(start_config): -1})

        # Add goal node to roadmap based on distance
        self.edge_dicts[r_id].update({-2: []})  # -2 -> goal config
        for node in self.node_lists[r_id]:
            if len(self.edge_dicts[r_id][-2]) == self.k2:
                break
            if self.distance(goal_config, node.q) <= self.maxdist and self.is_collision_free_edge(goal_config, node.q, r_id):
                self.edge_dicts[r_id][-2].append(node.id)
                self.edge_dicts[r_id][node.id].append(-2)
        self.node_id_q_dicts[r_id].update({-2: tuple(goal_config)})
        self.node_q_id_dicts[r_id].update({tuple(goal_config): -2})
        return
        
    def delete_start_goal_nodes(self, r_id, start_config, goal_config, start_node, goal_node):
        self.edge_dicts[r_id].pop(-1)
        self.edge_dicts[r_id].pop(-2)
        self.edge_dicts[r_id][start_node.id].remove(-1)
        self.edge_dicts[r_id][goal_node.id].remove(-2)
        self.node_id_q_dicts[r_id].pop(-1)
        self.node_id_q_dicts[r_id].pop(-2)
        self.node_q_id_dicts[r_id].pop(tuple(start_config))
        self.node_q_id_dicts[r_id].pop(tuple(goal_config))
        return

    def discretize_path_in_time(self, r_id, id_path):
        # Initialize the discretized path    
        discretized_path = {}

        # Calculate the start and end time of the first interval
        interval_start_t = 0.0
        interval_end_t = 0.0 

        # Discretize the intervals in id_path
        for i in range(len(id_path) - 1):
            interval_start_t = interval_end_t
            start_config = np.array(self.node_id_q_dicts[r_id][id_path[i]])
            end_config = np.array(self.node_id_q_dicts[r_id][id_path[i+1]])
            interval_end_t = interval_start_t + np.linalg.norm(end_config - start_config) / self.local_step * self.time_step
            first_multiple_of_timestep = np.round(np.ceil(interval_start_t / self.time_step) * self.time_step, 1)
            for t in np.arange(first_multiple_of_timestep, interval_end_t, self.time_step):
                t = np.round(t, 1)
                q = self.find_q_in_q_t_interval_given_t(np.append(start_config, interval_start_t), np.append(end_config, interval_end_t), t)
                discretized_path[t] = q

        # Add the goal configuration to the discretized path
        discretized_path[interval_end_t] = end_config

        return discretized_path

    #  Astar search
    def a_star_search(self, r_id, constraints):
        # constants
        start_id = -1
        goal_id = -2

        # compute set of conflict times for more efficient collision checking
        conflict_times = set()
        constraints_from_t = {constraint.t: [] for constraint in constraints}
        for constraint in constraints:
            conflict_times.add(constraint.t)
            constraints_from_t[constraint.t].append(constraint)
    
        came_from = {}
        g_score = {node_id: np.inf for node_id in self.edge_dicts[r_id].keys()}
        g_score[start_id] = 0
        f_score = {node_id: np.inf for node_id in self.edge_dicts[r_id].keys()}
        f_score[start_id] = self.heuristic(start_id, goal_id, r_id)

        unvisited_nodes = set(self.edge_dicts[r_id].keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Get the node with the lowest f score
            current_id = min(unvisited_nodes, key=lambda node_id: f_score[node_id])
            unvisited_nodes.remove(current_id)

            if g_score[current_id] == np.inf: # Remaining nodes are unreachable
                return [], np.inf # No path found

            if current_id == goal_id: # Check if the goal has been reached
                return self.reconstruct_path(current_id, came_from), g_score[goal_id] # Path found, reconstruct the path from start to goal

            # Check each neighbor of the current node
            for neighbor_id in self.edge_dicts[r_id][current_id]:
                if not self.transition_valid(r_id, g_score[current_id], current_id, neighbor_id, conflict_times, constraints_from_t):
                    continue
                
                current_q = self.node_id_q_dicts[r_id][current_id]
                neighbor_q = self.node_id_q_dicts[r_id][neighbor_id]
                # Calculate the distance between current node and neighbor
                distance = self.distance(current_q, neighbor_q)

                # Update the distance from start to neighbor
                new_distance = g_score[current_id] + distance
                if new_distance < g_score[neighbor_id]:
                    g_score[neighbor_id] = new_distance
                    f_score[neighbor_id] = new_distance + self.heuristic(neighbor_id, goal_id, r_id)
                    came_from[neighbor_id] = current_id
        if g_score[goal_id] == np.inf:
            return [], np.inf # No path found
        else: 
            return self.reconstruct_path(goal_id, came_from), g_score[goal_id] # Path found, reconstruct the path from start to goal

    def heuristic(self, node_id1, node_id2, r_id):
        q1 = np.array(self.node_id_q_dicts[r_id][node_id1])
        q2 = np.array(self.node_id_q_dicts[r_id][node_id2])
        return self.distance(q1, q2)
    
    def reconstruct_path(self, current_id, came_from):
        path = [current_id]
        while current_id in came_from.keys():
            current_id = came_from[current_id]
            path.append(current_id)
        return path[::-1]
    
    def compute_total_path_length(self, paths):
        total_length = 0
        for r_id, path in paths.items():
            path_length = 0
            for t, q in path.items():
                if t == 0:
                    continue
                q_prev = path[np.round(t-self.time_step, 1)]
                path_length += self.distance(q_prev, q, r_id)
            total_length += path_length
        return total_length

class ProbabilisticRoadMap:
    def __init__(self, environment, r_id, c_space, maxdist, k1=20, k2=10, build_type='kdtree', n=100, t=10, time_step=0.1, local_step=0.05) -> None:
        self.r_id = r_id
        self.c_space = c_space
        self.maxdist = maxdist
        self.k1 = k1
        self.k2 = k2
        self.build_type = build_type
        self.n = n
        self.t = t
        self.nodes = []
        self.edges = {}
        self.node_id_q_dict = {}
        self.node_q_id_dict = {}

        # collision checking
        self.time_step = time_step
        self.local_step = local_step

        # unpack environmental data
        self.env = environment
        self.agents = environment.agents                # This PRM class expects only 1 agent/robot in environment 
        self.robot_models = environment.robot_models    # This PRM class expects only 1 agent/robot in environment
        self.obstacles = environment.obstacles

        # call method to generate roadmap
        self.generate_roadmap()
    
    def generate_roadmap(self):
        # Clear roadmap
        self.nodes = []
        self.edges = {}

        # Build roadmap based on time
        if self.build_type == 't':
            start_time = time.time()
            while time.time() - start_time < self.t:
                sample = np.random.uniform([interval.lower for interval in self.c_space], 
                                            [interval.upper for interval in self.c_space], 
                                            len(self.c_space)
                                            )
                sample = np.round(sample, 2)
                new_node = Node(len(self.nodes), sample)
                self.nodes.append(new_node)
                self.edges.update({new_node.id: []})
                self._find_and_connect_neighbors(new_node)
            self.node_id_q_dict = self.generate_node_id_q_dict(self.nodes)
            self.node_q_id_dict = self.generate_node_q_id_dict(self.nodes)
            return
        
        # build roadmap based on number of nodes
        elif self.build_type == 'n':
            for i in range(self.n):
                sample = np.random.uniform([interval.lower for interval in self.c_space], 
                                            [interval.upper for interval in self.c_space], 
                                            len(self.c_space)
                                            )
                sample = np.round(sample, 2)
                new_node = Node(i, sample)
                self.nodes.append(new_node)
                self.edges.update({new_node.id: []})
                self._find_and_connect_neighbors(new_node)
            self.node_id_q_dict = self.generate_node_id_q_dict(self.nodes)
            self.node_q_id_dict = self.generate_node_q_id_dict(self.nodes)
            return
        
        # build roadmap based on kdtree, nodes are set a priori, no update of tree possible once created
        elif self.build_type == 'kdtree':

            # sample nodes
            samples = np.random.uniform([interval.lower for interval in self.c_space],
                                        [interval.upper for interval in self.c_space],
                                        (self.n, len(self.c_space))
                                        )
            samples = np.round(samples, 2)
            self.nodes = [Node(i, sample) for i, sample in enumerate(samples)]
            self.node_id_q_dict = self.generate_node_id_q_dict(self.nodes)
            self.node_q_id_dict = self.generate_node_q_id_dict(self.nodes)

            # create roadmap
            self.edges = {node.id: [] for node in self.nodes}
            tree = KDTree(samples)  
            for i, node in enumerate(self.nodes):
                id = node.id
                q = node.config
                # neighbors = tree.query_ball_point(q, r=self.maxdist)
                distances, neighbor_idxs = tree.query(q, k=self.k1+1) # +1 to exclude the node itself
                for d, id2 in zip(distances, neighbor_idxs):
                    if d > 0 \
                    and d < self.maxdist \
                    and len(self.edges[id]) < self.k1 \
                    and self.collision_free_edge(q, self.node_id_q_dict[id2]):
                        self.edges[id].append(id2)

        # raise error if build_type is not 't' or 'n' or 'kdtree'
        else:
            raise ValueError(f"Unexpected build_type {self.build_type}. Expected 't' or 'n' or 'kdtree'.")

    def query(self, start_q, goal_q):
        # Placeholder for search algorithm implementation
        pass

    def _find_and_connect_neighbors(self, new_node):
        candidate_neighbors = []
        distances = [(self.distance(new_node.config, node.config), node) 
                     for node in self.nodes[:-1]]
        distances.sort()  # Sort by distance
        
        for distance, neighbor in distances:
            if distance > self.maxdist or len(candidate_neighbors) == self.k1:
                break
            if not self._same_component(new_node, neighbor):
                candidate_neighbors.append(neighbor)
                
        for neighbor in candidate_neighbors:
            self._try_connect(new_node, neighbor)

    def _same_component(self, node1, node2):
        return False
    
    def _try_connect(self, node1, node2):
        if self.is_collision_free(node1, node2):
            self.edges[node1.id].append(node2.id)
            self.edges[node2.id].append(node1.id)
    
    def collision_free_edge(self, q1, q2):
        # Placeholder for collision check implementation with static obstacles in environment
        return True
    
    def generate_node_id_q_dict(self, nodes):
        return {node.id: node.config for node in nodes}
    
    def generate_node_q_id_dict(self, nodes):
        return {tuple(node.config): node.id for node in nodes}
    
    def distance(self, q1, q2):
        # Placeholder for distance calculation between two configurations
        return np.linalg.norm(np.array(q1) - np.array(q2))
    
    def add_start_and_goal(self, start_config, goal_config):
        raise NotImplementedError()
        # Ensure start and goal are numpy arrays for distance calculations
        start_node = np.array(start_config)
        self.start_nodes.append(start_node)
        goal_node = np.array(goal_config)
        self.goal_nodes.append(goal_node)

        # Initialize minimum distances to a large value
        min_dist_start, min_dist_goal = np.inf, np.inf
        nearest_start, nearest_goal = None, None
        
        # Iterate over each node to find the nearest node to start and goal
        for node in self.nodes:
            dist_to_start = np.linalg.norm(node - start_node)
            dist_to_goal = np.linalg.norm(node - goal_node)

            # Update nearest start node
            if dist_to_start < min_dist_start and self.is_collision_free(start_node, node, self.config_space):
                min_dist_start = dist_to_start
                nearest_start = node

            # Update nearest goal node
            if dist_to_goal < min_dist_goal and self.is_collision_free(goal_node, node, self.config_space):
                min_dist_goal = dist_to_goal
                nearest_goal = node

        # Add start and goal nodes to the roadmap if they have a nearest node within max_edge_len
        if nearest_start is not None:
            self.knn_dict[tuple(start_node)] = [nearest_start.tolist()]
            self.knn_dict[tuple(nearest_start)].append(start_node.tolist())

        if nearest_goal is not None:
            self.knn_dict[tuple(goal_node)] = [nearest_goal.tolist()]
            self.knn_dict[tuple(nearest_goal)].append(goal_node.tolist())

    def visualize_roadmap(self, solution=None):
        raise NotImplementedError()
        fig, ax = plt.subplots()
        
        # Plot nodes as blue dots
        for node in self.nodes:
            ax.plot(node[0], node[1], 'o', color='blue')
        
        # Draw edges between each node and its neighbors
        for node, neighbors in self.knn_dict.items():
            for neighbor in neighbors:
                ax.plot([node[0], neighbor[0]], [node[1], neighbor[1]], 'k-', linewidth=0.5)
        
        # Highlight start and goal nodes
        for start_node in self.start_nodes:
            ax.plot(start_node[0], start_node[1], 'o', color='green', markersize=10)  # Larger green dots for start nodes
        for goal_node in self.goal_nodes:
            ax.plot(goal_node[0], goal_node[1], 'o', color='red', markersize=10)  # Larger red dots for goal nodes

        # Plot the solution paths if provided
        if solution is not None:
            for agent_id, path in solution.items():
                path_locs = [step['loc'] for step in path]
                xs, ys = zip(*path_locs)
                ax.plot(xs, ys, marker='o', linestyle='-', label=f"Path for {agent_id}")
        
        ax.set_aspect('equal', adjustable='box')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Roadmap Visualization')
        plt.legend()
        plt.show()

    #  Astar search
    def a_star_search(self, constraints):
        # constants
        start_id = -1
        goal_id = -2

        # compute set of conflict times for more efficient collision checking
        conflict_times = set()
        constraints_from_t = {constraint.t: [] for constraint in constraints}
        for constraint in constraints:
            conflict_times.add(constraint.t)
            constraints_from_t[constraint.t].append(constraint)
    
        came_from = {}
        g_score = {node.id: np.inf for node in self.nodes}
        g_score[self.start_id] = 0
        f_score = {node.id: np.inf for node in self.nodes}
        f_score[self.start_id] = self.heuristic(self.start_id, self.goal_id)

        unvisited_nodes = set(self.edges.keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Get the node with the lowest f score
            current_id = min(unvisited_nodes, key=lambda node_id: f_score[node_id])
            unvisited_nodes.remove(current_id)

            if g_score[current_id] == np.inf: # Remaining nodes are unreachable
                return [], np.inf # No path found

            if current_id == goal_id: # Check if the goal has been reached
                return self.reconstruct_path(current_id, came_from) # Path found, reconstruct the path from start to goal

            # Check each neighbor of the current node
            for neighbor_id in self.edges[current_id]:
                if not self.transition_valid(current_id, neighbor_id, conflict_times, constraints_from_t):
                    continue

                # Calculate the distance between current node and neighbor
                distance = self.distance(current_id, neighbor_id)

                # Update the distance from start to neighbor
                new_distance = g_score[current_id] + distance
                if new_distance < g_score[neighbor_id]:
                    g_score[neighbor_id] = new_distance
                    f_score[neighbor_id] = new_distance + self.heuristic(neighbor_id, goal_id)
                    came_from[neighbor_id] = current_id
        if g_score[goal_id] == np.inf:
            return [], np.inf # No path found
        else: 
            return self.reconstruct_path(goal_id, came_from), g_score[goal_id] # Path found, reconstruct the path from start to goal

    def heuristic(self, node_id1, node_id2):
        q1 = np.array(self.node_id_q_dict[node_id1])
        q2 = np.array(self.node_id_q_dict[node_id2])
        return self.distance(q1, q2)
    
    def reconstruct_path(self, current_id, came_from):
        path = [current_id]
        while current_id in came_from.keys():
            current_id = came_from[current_id]
            path.append(current_id)
        return path[::-1]
    
    def transition_valid(self, d_to_n_1, n_1_id, n_2_id, conflict_times, constraints_from_t):
        # Get the configurations for the current node and the neighbor node
        q1 = np.array(self.node_id_q_dict[n_1_id])
        q2 = np.array(self.node_id_q_dict[n_2_id])

        # Calculate the time interval between the current node and the neighbor node
        t1 = float(d_to_n_1 / self.local_step) * self.time_step
        t2 = t1 + np.linalg.norm(q2 - q1) / self.local_step * self.time_step
        delta_t = t2 - t1

        # Append the start time to the current and neighbor configurations
        q1_t1 = np.append(q1, t1)
        q2_t2 = np.append(q2, t2)

        # Calculate the start and end times for the interval
        interval_t1 = np.round(np.ceil(t1 / self.time_step) * self.time_step, 1)
        interval_t2 = np.round(np.floor(t2 / self.time_step) * self.time_step, 1)

        # Discretize the path from the start configuration to the end configuration
        discretized_path = {}
        for t in np.round(np.arange(interval_t1, interval_t2 + self.time_step, self.time_step), 1):   # np_arange(start, stop) -> [start, stop) 
            q_at_t = self.find_q_in_q_t_interval_given_t(q1_t1, q2_t2, t)
            discretized_path.update({np.round(t, 1): q_at_t})
            if t in conflict_times: 
                constraints = constraints_from_t[t]
                for constraint in constraints:
                    r_2_q = constraint.q2
                    if self.robot_robot_collision(q_at_t, r_2_q, constraint.r_2_id):
                        return False       
        return True
    
    def find_q_in_q_t_interval_given_t(self, q1_t1, q2_t2, t):
        delta_t = q2_t2[-1] - q1_t1[-1]
        x = (t - q1_t1[-1]) / delta_t
        return np.round(q1_t1[:-1] + (q2_t2[:-1] - q1_t1[:-1]) * x, 2)
    
    def robot_robot_collision(self, r_1_q, r_2_q, r_2_id):
        # Set the pose of the robot model in pybullet to location for collision checking
        self.robot_models[self.r_id].set_pose(self.robot_models[self.r_id].joints, tuple(r_1_q))
        self.robot_models[r_2_id].set_pose(self.robot_models[r_2_id].joints, tuple(r_2_q))
        # Check for collision between the two robots
        closest_points = p.getClosestPoints(bodyA=self.robot_models[self.r_id].robot_id, bodyB=self.robot_models[r_2_id].robot_id, distance=0.0)
        if closest_points:
            return True
        return False
    
if __name__ == "__main__":
    main()