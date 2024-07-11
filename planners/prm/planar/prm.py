"""

PRM Planner for Planar Robots

Note: This PRM class cannot be used as a standalone planner BUT
is a blueprint for more specific PRM classes that inherit from it. 

"""

from collections import deque
import time
from matplotlib import pyplot as plt
import numpy as np

from planners.prm.planar.utils import Node, INF


class PRM: 
    def __init__(self, environment, local_step) -> None:
        self.env = environment
        self.agents = environment.agents                # This PRM class expects only 1 agent/robot in environment 
        self.robot_models = environment.robot_models    # This PRM class expects only 1 agent/robot in environment
        self.config_space = self.robot_models[0].config_space
        self.obstacles = environment.obstacles

        self.local_step = local_step

        self.nodes = [] 

        self.node_id_config_dict = {}
        self.node_config_id_dict = {}

        self.roadmap = {} #self.generate_neighbor_dict(self.nodes) # dict linking each node to neighbors using node indices

    # dictionaries for mapping id's and configurations
    def update_node_id_config_dict(self, nodes):
        for node in nodes:
            self.node_id_config_dict.update({node.id: node.q})
        return
    
    def update_node_config_id_dict(self, nodes):
        for node in nodes:
            self.node_config_id_dict.update({node.q: node.id})
        return
    
    # # Learning
    # sampling functions
    def generate_sample(self):
        sample = np.random.uniform(
            [interval.lower for interval in self.config_space],
            [interval.upper for interval in self.config_space],
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

    # distance metric
    def distance(self, q1, q2):
        raise NotImplementedError()

    def specific_distance(self, q1, q2):
        pos1 = self.robot_models[0].forward_kinematics(q1)
        pos2 = self.robot_models[0].forward_kinematics(q2)
        return self.robot_models[0].distance(pos1, pos2)
    
    def general_distance(self, q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))
    
    # local planner and collision checking
    def is_collision_free_sample(self, sample):
        robot = self.robot_models[0]
        # within map bounds
        if not self.robot_in_env_bounds(sample, robot):
            return False
        # self collisions
        if self.robot_self_collision(sample, robot):
            return False
        # collisions with obstacles
        for obstacle in self.obstacles:
            # sample1 = sample[i*robot1.n_links:(i+1)*robot1.n_links]
            if self.robot_obstacle_collision(sample, robot, obstacle):
                return False
        return True
    
    # def is_collision_free_sample(self, sample):
    #     for i, robot1 in enumerate(self.robot_models): 
    #         config1 = sample[i*robot1.n_links:(i+1)*robot1.n_links]
    #         # within map bounds
    #         if not self.robot_in_env_bounds(config1, robot1):
    #             return False
    #         # self collisions
    #         if self.robot_self_collision(config1, robot1):
    #             return False
    #         # collisions with other robots
    #         for j, robot2 in enumerate(self.robot_models[i+1:]): 
    #             # config1 = sample[i*robot1.n_links:(i+1)*robot1.n_links]
    #             config2 = sample[(i+j+1)*robot2.n_links:(i+j+2)*robot2.n_links]
    #             sample12 = self.merge_configs([config1, config2])
    #             if self.robot_robot_collision(sample12, robot1, robot2):
    #                 return False
    #         for obstacle in self.obstacles:
    #             # sample1 = sample[i*robot1.n_links:(i+1)*robot1.n_links]
    #             if self.robot_obstacle_collision(config1, robot1, obstacle):
    #                 return False
    #     return True
    
    def robot_in_env_bounds(self, config, robot):
        robot.set_pose(config)
        return self.env.robot_in_env_bounds(robot)
    
    def robot_self_collision(self, config, robot):
        robot.set_pose(config)
        return self.env.robot_self_collision(robot)
    
    # def robot_robot_collision(self, sample, robot1, robot2):
    #     sample1, sample2 = self.split_config(sample)
    #     robot1.set_pose(sample1)
    #     robot2.set_pose(sample2)
    #     return self.env.robot_robot_collision(robot1, robot2)
    
    def robot_obstacle_collision(self, sample, robot, obstacle):
        robot.set_pose(sample)
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

    # # Querying roadmap
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
    
    def query(self):
        path = []
        paths = {}
        start_config = self.agents[0]['start']
        goal_config = self.agents[0]['goal']

        if not self.is_collision_free_sample(start_config): 
            print("Start configuration is in collision.")
            return path
        
        if not self.is_collision_free_sample(goal_config):
            print("Goal configuration is in collision.")
            return path
        
        # Find the nearest roadmap nodes to the start and goal configurations
        start_node = self.find_nearest_roadmap_node(start_config)
        goal_node = self.find_nearest_roadmap_node(goal_config)

        # Use Dijkstra to find a path between the nearest start and goal nodes
        dijkstra_start_time = time.perf_counter()
        d_path, d_distance = self.dijkstra_search(start_node.id, goal_node.id)
        dijkstra_duration = time.perf_counter() - dijkstra_start_time
        print(f"Dijkstra: Composite path in {dijkstra_duration:.6f} seconds with distance {d_distance:.2f}")
        
        # Use A* to find a path between the nearest start and goal nodes
        a_star_start_time = time.perf_counter()
        a_path, a_distance = self.a_star_search(start_node.id, goal_node.id)
        a_star_duration = time.perf_counter() - a_star_start_time
        print(f"A*: Composite path in {a_star_duration:.6f} seconds with distance {a_distance:.2f}")
        # Store the path for the agent
        path = a_path
        paths = {self.agents[0]['name']: [self.node_id_config_dict[node_id] for node_id in path]}
        return paths, path
    
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
                neighbor_config = self.node_id_config_dict[neighbor_id]
                current_node_config = self.node_id_config_dict[current_node_id]
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
                neighbor_config = self.node_id_config_dict[neighbor_id]
                current_node_config = self.node_id_config_dict[current_node_id]
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
        config1 = np.array(self.node_id_config_dict[node_id1])
        config2 = np.array(self.node_id_config_dict[node_id2])
        return self.distance(config1, config2)

    # compute
    def compute_avg_degree(self):
        return sum([len(neighbors) for neighbors in self.roadmap.values()]) / len(self.roadmap)
    
    def compute_nr_nodes(self):
        return len(self.nodes)
    
    def compute_nr_edges(self):
        return sum([len(neighbors) for neighbors in self.roadmap.values()]) / 2

    # Plot roadmap 
    def plot_roadmap_2D_subspace(self):
        fig, ax = plt.subplots() 

        for node, neighbors in self.roadmap.items():
            node_q = self.node_id_config_dict[node]
            for neighbor in neighbors:
                neighbor_q = self.node_id_config_dict[neighbor]
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
        ax.set_xlim([self.config_space[0].lower, self.config_space[0].upper])
        ax.set_ylim([self.config_space[1].lower, self.config_space[1].upper])
        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks
        plt.show()