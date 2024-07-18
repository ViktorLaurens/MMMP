"""

Coupled PRM Planner for Planar Robots

Notes: 

 - BLUEPRINT: This PRM class cannot be used as a standalone planner BUT
is a blueprint for more specific PRM classes that inherit from it.

 - MULTI ROBOT USE: This PRM class can handle 1 or more robots in a single planar environment.

 """

from collections import deque
import time
from matplotlib import pyplot as plt
import numpy as np

from planners.prm.pybullet.utils import INF, Node
from robots.panda import Panda

class CoupledPRM: 
    def __init__(self, environment, time_step=0.1, local_step=0.05) -> None:
        # unpack environmental data
        self.env = environment
        self.agents = environment.agents                
        self.robot_models = environment.robot_models    
        self.obstacles = environment.obstacles

        # robot handling
        self.c_space = self.create_composite_c_space(environment.robot_models) # composite configuration space

        # time step and local step
        self.time_step = time_step
        self.local_step = local_step

        # Create data structures for roadmap handling
        self.nodes = []
        self.roadmap = {}
        self.node_q_id_dict = {}
        self.node_id_q_dict = {}

        # self.n_knn = n_knn
        # self.max_edge_len = max_edge_len
    
    # methods for coupled PRM
    def create_composite_c_space(self, robot_models):
        """Creates a composite configuration space by concatenating the config_space of all robots."""
        composite_space = []
        for model in robot_models:
            if isinstance(model, Panda):
                composite_space.extend(model.arm_c_space)
            else:
                composite_space.extend(model.c_space)
        return composite_space
    
    def merge_configs(self, configs):
        """Merges multiple configurations into a single higher-dimensional configuration."""
        merged_config = np.concatenate([np.array(config) for config in configs])
        return np.array(merged_config)
    
    def split_config(self, config):
        """Splits a higher-dimensional configuration into individual robot configurations."""
        split_configs = []
        offset = 0
        for robot in self.robot_models:
            if isinstance(robot, Panda):
                dim = len(robot.arm_c_space)
            else:
                dim = len(robot.c_space)
            split_configs.append(np.array(config[offset:offset+dim]))
            offset += dim
        return split_configs
    
    # Other attribute methods
    def update_node_id_q_dict(self, nodes):
        for node in nodes:
            self.node_id_q_dict.update({node.id: node.q})
        return
    
    def update_node_q_id_dict(self, nodes):
        for node in nodes:
            self.node_q_id_dict.update({node.q: node.id})
        return
    
    # # Learning
    # sampling functions
    def generate_sample(self):
        sample = np.random.uniform(
            [interval.lower for interval in self.c_space],
            [interval.upper for interval in self.c_space],
        )
        sample = np.round(sample, 2)
        return sample
    
    def generate_free_sample(self):
        sample = self.generate_sample()
        while not self.is_collision_free_sample(sample):
            sample = self.generate_sample()
        return sample
    
    def generate_free_samples(self, n): 
        # Sample n free nodes
        sampled_free_nodes = []
        for _ in range(n):
            sample = self.generate_free_sample()
            node = Node(id=len(self.nodes)+len(sampled_free_nodes), q=sample)
            sampled_free_nodes.append(node)
        return sampled_free_nodes
    
    # building roadmap    
    def build_roadmap_n(self, n):
        raise NotImplementedError()
    
    def build_roadmap_time(self, t):
        raise NotImplementedError()
    
    def add_n_samples(self, n):
        raise NotImplementedError()
    
    def add_build_period(self, t):
        raise NotImplementedError()

    def update_roadmap(self): 
        raise NotImplementedError()

    # distance
    def distance(self, q1, q2):
        raise NotImplementedError()
    
    def general_distance(self, q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))
    
    def specific_distance(self, q1, q2):
        split_configs1 = self.split_config(q1)
        split_configs2 = self.split_config(q2)
        dist = 0
        for i, robot in enumerate(self.robot_models):
            dist += robot.distance_metric(split_configs1[i], split_configs2[i])
        return dist
    
    # local planner 
    def is_collision_free_sample(self, sample):
        for i, robot1 in enumerate(self.robot_models): 
            config1 = sample[i*robot1.arm_dimension:(i+1)*robot1.arm_dimension]
            # self collisions
            if self.robot_self_collision(config1, robot1):
                return False
            # collisions with other robots
            for j, robot2 in enumerate(self.robot_models[i+1:]): 
                # config1 = sample[i*robot1.arm_dimension:(i+1)*robot1.arm_dimension]
                config2 = sample[(i+j+1)*robot2.arm_dimension:(i+j+2)*robot2.arm_dimension]
                sample12 = self.merge_configs([config1, config2])
                if self.robot_robot_collision(sample12, robot1, robot2):
                    return False
            for obstacle in self.obstacles:
                # sample1 = sample[i*robot1.arm_dimension:(i+1)*robot1.arm_dimension]
                if self.robot_obstacle_collision(config1, robot1, obstacle):
                    return False
        return True
    
    def robot_self_collision(self, q, robot):
        robot.set_arm_pose(q)
        return self.env.robot_self_collision(robot)
    
    def robot_robot_collision(self, sample, robot1, robot2):
        sample1, sample2 = self.split_config(sample)
        robot1.set_arm_pose(sample1)
        robot2.set_arm_pose(sample2)
        return self.env.robot_robot_collision(robot1, robot2)
    
    def robot_obstacle_collision(self, q, robot, obstacle):
        robot.set_arm_pose(q)
        return self.env.robot_obstacle_collision(robot, obstacle)
    
    def is_collision_free_node(self, node):
        return self.is_collision_free_sample(node.q)
    
    def is_collision_free_edge(self, s, g):
        # Check for collisions along the line segment between s and g
        s = np.array(s)
        g = np.array(g)
        for t in np.arange(0, 1, self.local_step):
            sample = s + t * (g - s)
            if not self.is_collision_free_sample(sample):
                return False
        return True
    
    # Querying roadmap
    def query(self):
        path = []
        paths = {}

        start_config = np.array([])
        goal_config = np.array([])
        for agent in self.agents:
            # find paths from nearest node to start and nearest node to goal 
            start_config = np.append(start_config, agent['start'])
            goal_config = np.append(goal_config, agent['goal'])

        if not self.is_collision_free_sample(start_config): 
            print("Start configuration is in collision.")
            return paths
        if not self.is_collision_free_sample(goal_config):
            print("Goal configuration is in collision.")
            return paths
        
        # Find the nearest roadmap nodes to the start and goal configurations
        start_node = self.find_nearest_roadmap_node(start_config)
        goal_node = self.find_nearest_roadmap_node(goal_config)

        # # Use Dijkstra to find a path between the nearest start and goal nodes
        # dijkstra_start_time = time.perf_counter()
        # d_path, d_distance = self.dijkstra_search(start_node.id, goal_node.id)
        # dijkstra_duration = time.perf_counter() - dijkstra_start_time
        # print(f"Dijkstra: Composite path in {dijkstra_duration:.6f} seconds with distance {d_distance:.2f}")
        
        # Use A* to find a path between the nearest start and goal nodes
        a_star_start_time = time.perf_counter()
        a_path, a_distance = self.a_star_search(start_node.id, goal_node.id)
        a_star_duration = time.perf_counter() - a_star_start_time
        print(f"A*: Composite path in {a_star_duration:.6f} seconds with distance {a_distance:.2f}")
        
        # Store the path for the agent
        id_path = a_path
        return self.st_path_from_id_path(id_path)
        for config in path:
            split_configs = self.split_config(self.node_id_q_dict[config])
            for i, agent in enumerate(self.agents):
                if agent['name'] not in paths:
                    paths[agent['name']] = []
                paths[agent['name']].append(tuple(split_configs[i]))
        return paths
    
    def find_nearest_roadmap_node(self, config):
        """Finds the nearest roadmap node to a given configuration by looping over all nodes."""
        min_dist = float('inf')
        nearest_node = None
        for node in self.nodes:
            dist = self.distance(config, node.q)
            if dist < min_dist and self.is_collision_free_edge(config, node.q):
                min_dist = dist
                nearest_node = node
        return nearest_node
    
    def st_path_from_id_path(self, id_path):
        """Converts a path of node IDs to a path of configurations of the structure: {r_id: {t: q}}."""
        paths = [[np.array(agent["start"])] for agent in self.agents]
        for node_id in id_path:
            composite_q = self.node_id_q_dict[node_id]
            qs = self.split_config(composite_q)
            for i in range(len(qs)):
                paths[i].append(qs[i])
        for i, agent in enumerate(self.agents):
            paths[i].append(np.array(agent["goal"]))

        st_path = {}
        for i, model in enumerate(self.robot_models):
            st_path[model.r_id] = self.discretize_path_in_time(paths[i])
        return st_path
    
    def discretize_path_in_time(self, q_path):
        # Initialize the discretized path    
        discretized_path = {}

        # Calculate the start and end time of the first interval
        interval_start_t = 0.0
        interval_end_t = 0.0 

        # Discretize the intervals in id_path
        for i in range(len(q_path) - 1):
            interval_start_t = interval_end_t
            start_config = np.array(q_path[i])
            end_config = np.array(q_path[i+1])
            interval_end_t = interval_start_t + np.linalg.norm(end_config - start_config) / self.local_step * self.time_step
            first_multiple_of_timestep = np.round(np.ceil(interval_start_t / self.time_step) * self.time_step, 1)
            for t in np.arange(first_multiple_of_timestep, interval_end_t, self.time_step):
                t = np.round(t, 1)
                q = self.find_q_in_q_t_interval_given_t(np.append(start_config, interval_start_t), np.append(end_config, interval_end_t), t)
                discretized_path[t] = q

        # Add the goal configuration to the discretized path
        discretized_path[interval_end_t] = end_config
        return discretized_path
    
    def find_q_in_q_t_interval_given_t(self, q1_t1, q2_t2, t):
        delta_t = q2_t2[-1] - q1_t1[-1]
        x = (t - q1_t1[-1]) / delta_t
        return np.round(q1_t1[:-1] + (q2_t2[:-1] - q1_t1[:-1]) * x, 2)
    
    # Search methods
    def dijkstra_search(self, start_node_id, goal_node_id):
        """Perform Dijkstra's algorithm using node IDs."""
        # Initialize distances and previous node tracking
        distance_from_start = {node_id: INF for node_id in self.roadmap.keys()}
        distance_from_start[start_node_id] = 0
        previous_node = {node_id: None for node_id in self.roadmap.keys()}

        unvisited_nodes = set(self.roadmap.keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Choose the unvisited node with the smallest distance
            current_node_id = min(unvisited_nodes, key=lambda node_id: distance_from_start[node_id])
            unvisited_nodes.remove(current_node_id)

            if distance_from_start[current_node_id] == INF:
                break  # Remaining nodes are unreachable

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            for neighbor_id in self.roadmap[current_node_id]:
                neighbor_config = self.node_id_q_dict[neighbor_id]
                current_node_config = self.node_id_q_dict[current_node_id]
                # Calculate the distance between current node and neighbor
                distance = self.distance(neighbor_config, current_node_config)

                new_distance = distance_from_start[current_node_id] + distance
                if new_distance < distance_from_start[neighbor_id]:
                    distance_from_start[neighbor_id] = new_distance
                    previous_node[neighbor_id] = current_node_id

        # Reconstruct the path from start_node_id to goal_node_id
        path = deque()
        current_node_id = goal_node_id
        if previous_node[current_node_id] is None:
            return list(path), INF  # No path found 
        else:
            while previous_node[current_node_id] is not None:
                path.appendleft(current_node_id)
                current_node_id = previous_node[current_node_id]
            path.appendleft(start_node_id)
            return list(path), distance_from_start[goal_node_id]

    def a_star_search(self, start_node_id, goal_node_id):
        """Perform A* search using node IDs."""
        # Initialize distances and previous node tracking
        distance_from_start = {node_id: INF for node_id in self.roadmap.keys()}
        distance_from_start[start_node_id] = 0
        estimated_total_cost = {node_id: INF for node_id in self.roadmap.keys()}
        estimated_total_cost[start_node_id] = self.heuristic(start_node_id, goal_node_id)
        previous_node = {node_id: None for node_id in self.roadmap.keys()}

        unvisited_nodes = set(self.roadmap.keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Choose the unvisited node with the smallest estimated total cost
            current_node_id = min(unvisited_nodes, key=lambda node_id: estimated_total_cost[node_id])
            unvisited_nodes.remove(current_node_id)

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            for neighbor_id in self.roadmap[current_node_id]:
                neighbor_config = self.node_id_q_dict[neighbor_id]
                current_node_config = self.node_id_q_dict[current_node_id]
                # Calculate the distance between current node and neighbor
                distance = self.distance(neighbor_config, current_node_config)

                new_distance = distance_from_start[current_node_id] + distance
                if new_distance < distance_from_start[neighbor_id]:
                    distance_from_start[neighbor_id] = new_distance
                    estimated_total_cost[neighbor_id] = new_distance + self.heuristic(neighbor_id, goal_node_id)
                    previous_node[neighbor_id] = current_node_id

        # Reconstruct the path from start_node_id to goal_node_id
        path = deque()
        current_node_id = goal_node_id
        if previous_node[current_node_id] is None:
            return list(path), INF  # No path found 
        else:
            while previous_node[current_node_id] is not None:
                path.appendleft(current_node_id)
                current_node_id = previous_node[current_node_id]
            path.appendleft(start_node_id)
            return list(path), distance_from_start[goal_node_id]

    def heuristic(self, node_id1, node_id2):
        """Calculate the distance between two nodes."""
        config1 = np.array(self.node_id_q_dict[node_id1])
        config2 = np.array(self.node_id_q_dict[node_id2])
        return self.distance(config1, config2)

    # compute
    def compute_combined_avg_degree(self):
        neighbor_sum = sum([len(neighbors) for neighbors in self.roadmap.values()])
        nodes_sum = len(self.roadmap)
        return neighbor_sum / nodes_sum
    
    def compute_combined_nr_nodes(self):
        return len(self.nodes)
    
    def compute_nr_edges(self, robot_id):
        return sum([len(neighbors) for neighbors in self.roadmaps[robot_id].values()]) / 2
    
    def compute_combined_nr_edges(self):
        return sum([len(neighbors) for neighbors in self.roadmap.values()]) / 2

    # Plot roadmap 
    def plot_roadmap_2D_subspace(self):
        fig, ax = plt.subplots() 

        for node, neighbors in self.roadmap.items():
            node_q = self.node_id_q_dict[node]
            for neighbor in neighbors:
                neighbor_q = self.node_id_q_dict[neighbor]
                ax.plot([node_q[0], neighbor_q[0]], [node_q[1], neighbor_q[1]], 'k-', linewidth=0.5)
        
        for node in self.nodes:
            ax.plot(node.q[0], node.q[1], 'o', color='blue', markersize=3)

        for agent in self.agents: 
            start_q = agent['start']
            # print(start_q)
            goal_q = agent['goal']
            # print(goal_q)
            ax.plot(start_q[0], start_q[1], 'o', color='red', markersize=4)
            ax.plot(goal_q[0], goal_q[1], 'o', color='green', markersize=4)
        
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim([self.c_space[0].lower, self.c_space[0].upper])
        ax.set_ylim([self.c_space[1].lower, self.c_space[1].upper])
        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks
        plt.show()