"""

Python implementation of coupled PRM for pybullet simulation

"""
from collections import deque
from itertools import combinations
import time

import numpy as np
from scipy.spatial import KDTree

from planners.prm.pybullet.utils import INF
from planners.prm.pybullet.utils import Node

class PRM: 
    def __init__(self, environment, local_step) -> None:
        self.env = environment
        self.agents = environment.agents                # This PRM class expects only 1 agent/robot in environment 
        self.robot_models = environment.robot_models    # This PRM class expects only 1 agent/robot in environment
        self.c_space = self.robot_models[0].arm_c_space
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

    # distance metric
    def distance(self, q1, q2):
        raise NotImplementedError()

    def general_distance(self, q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))
    
    def specific_distance(self, q1, q2):
        return self.robot_models[0].distance_metric(q1, q2)
    
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



class PRM:
    def __init__(self, environment, maxdist=None, k1=20, k2=10, build_type='kdtree', n=100, t=10, time_step=0.1, local_step=None) -> None:
        # unpack environmental data
        self.env = environment
        self.agents = environment.agents                
        self.robot_models = environment.robot_models    
        self.obstacles = environment.obstacles

        self.r_ids = [environment.robot_models[i].r_id for i in range(len(environment.robot_models))]
        self.config_spaces = self.create_config_spaces(environment.robot_models)
        self.maxdist = maxdist
        if self.maxdist is None:
            self.maxdist = self.calc_maxdist(self.r_ids, self.config_spaces, 5)
        self.k1 = k1
        self.k2 = k2
        self.build_type = build_type
        self.n = n
        self.t = t
        self.node_lists = self.create_node_lists(environment.robot_models)
        self.edge_dicts = self.create_edge_dicts(environment.robot_models)
        self.node_id_q_dicts = self.create_node_id_q_dicts(environment.robot_models)
        self.node_q_id_dicts = self.create_node_q_id_dicts(environment.robot_models)   

        # collision checking
        self.time_step = time_step
        self.local_step = local_step
        if self.local_step is None:
            self.local_step = self.maxdist / 10

        # call method to generate roadmap
        self.generate_roadmaps()

    #  Initialization methods
    def create_config_spaces(self, robot_models):
        config_spaces = []
        for model in robot_models:
            config_spaces.append(model.arm_c_space)
        return config_spaces
    
    def calc_maxdists(self, r_ids, c_spaces, res):
        assert len(r_ids) == len(c_spaces)
        maxdists = {}
        for r_id, c_space in zip(r_ids, c_spaces): 
            unit_vector = np.array([])
            for interval in c_space: 
                unit_vector = np.append(unit_vector, (interval.upper - interval.lower) / res + interval.lower)
            maxdists.update({r_id: self.distance(r_id, unit_vector, [interval.lower for interval in c_space])})
        return maxdists
    
    def calc_maxdist(self, r_ids, c_spaces, res):
        return np.average(list(self.calc_maxdists(r_ids, c_spaces, res).values()))
    
    def create_node_lists(self, robot_models):
        node_list = []
        for _ in robot_models:
            node_list.append([])
        return node_list
    
    def create_edge_dicts(self, robot_models):
        edge_dicts = []
        for _ in robot_models:
            edge_dicts.append({})
        return edge_dicts
    
    def create_node_id_q_dicts(self, robot_models):
        node_id_q_dicts = []
        for _ in robot_models:
            node_id_q_dicts.append({})
        return node_id_q_dicts
    
    def create_node_q_id_dicts(self, robot_models):
        node_q_id_dicts = []
        for _ in robot_models:
            node_q_id_dicts.append({})
        return node_q_id_dicts
    
    def generate_node_id_q_dict(self, nodes):
        return {node.id: node.q for node in nodes}
    
    def generate_node_q_id_dict(self, nodes):
        return {tuple(node.q): node.id for node in nodes}
    
    # learning methods
    def generate_roadmaps(self):
        for r_id in self.r_ids:
            self.generate_roadmap(r_id)
        return

    def generate_roadmap(self, r_id):
        # Build roadmap based on time
        r_index = self.r_ids.index(r_id)
        if self.build_type == 't':
            start_time = time.time()
            while time.time() - start_time < self.t:
                sample = np.random.uniform([interval.lower for interval in self.config_spaces[r_index]], 
                                            [interval.upper for interval in self.config_spaces[r_index]], 
                                            len(self.config_spaces[r_index])
                                            )
                sample = np.round(sample, 2)
                new_node = Node(len(self.node_lists[r_index]), sample)
                self.node_lists[r_index].append(new_node)
                self.edge_dicts[r_index].update({new_node.id: []})
                self._find_and_connect_neighbors(new_node)
            self.node_id_q_dicts[r_index] = self.generate_node_id_q_dict(self.node_lists[r_index])
            self.node_q_id_dicts[r_index] = self.generate_node_q_id_dict(self.node_lists[r_index])
            return
        # build roadmap based on number of nodes
        elif self.build_type == 'n':
            for i in range(self.n):
                sample = np.random.uniform([interval.lower for interval in self.config_spaces[r_index]], 
                                            [interval.upper for interval in self.config_spaces[r_index]], 
                                            len(self.config_spaces[r_index])
                                            )
                sample = np.round(sample, 2)
                new_node = Node(i, sample)
                self.node_lists[r_index].append(new_node)
                self.edge_dicts[r_index].update({new_node.id: []})
                self._find_and_connect_neighbors(r_id, new_node)
            self.node_id_q_dicts[r_index] = self.generate_node_id_q_dict(self.node_lists[r_index])
            self.node_q_id_dicts[r_index] = self.generate_node_q_id_dict(self.node_lists[r_index])
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
            self.node_lists[r_index] = [Node(i, sample) for i, sample in enumerate(samples)]
            self.node_id_q_dicts[r_index] = self.generate_node_id_q_dict(self.node_lists[r_index])
            self.node_q_id_dicts[r_index] = self.generate_node_q_id_dict(self.node_lists[r_index])

            # create roadmap
            self.edge_dicts[r_index] = {node.id: [] for node in self.node_lists[r_index]}
            tree = KDTree(samples)  
            for i, node in enumerate(self.node_lists[r_index]):
                id = node.id
                q = node.q
                # neighbors = tree.query_ball_point(q, r=self.maxdist)
                distances, neighbor_idxs = tree.query(q, k=self.k1+1) # +1 to exclude the node itself
                for d, id2 in zip(distances, neighbor_idxs):
                    # if >= k2: 
                    # break
                    if d > 0 \
                    and d < self.maxdist \
                    and len(self.edge_dicts[r_index][id]) < self.k2 \
                    and self.is_collision_free_edge(q, self.node_id_q_dicts[r_index][id2], r_id):
                        self.edge_dicts[r_index][id].append(id2)
            return
        # raise error if build_type is not 't' or 'n' or 'kdtree'
        else:
            raise ValueError(f"Unexpected build_type {self.build_type}. Expected 't' or 'n' or 'kdtree'.")

    def sample_valid(self, n, r_id):
        samples = []
        r_index = self.r_ids.index(r_id)
        while len(samples) < n: 
            sample = np.random.uniform([interval.lower for interval in self.config_spaces[r_index]], 
                                        [interval.upper for interval in self.config_spaces[r_index]], 
                                        len(self.config_spaces[r_index])
                                        )
            sample = np.round(sample, 2)
            if self.is_collision_free_sample(sample, r_id):
                samples.append(sample)
        return samples

    def _find_and_connect_neighbors(self, r_id, new_node):
        r_index = self.r_ids.index(r_id)
        candidate_neighbors = []
        distances = [(self.distance(r_id, new_node.config, node.config), node) 
                     for node in self.node_lists[r_index][:-1]]
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
        r_index = self.r_ids.index(r_id)
        if self.is_collision_free_edge(node1, node2):
            self.edge_dicts[r_index][node1.id].append(node2.id)
            self.edge_dicts[r_index][node2.id].append(node1.id)
    
    #  Distance metric
    # def distance(self, q1, q2):
    #     # Placeholder for distance calculation between two configurations
    #     return np.linalg.norm(np.array(q1) - np.array(q2))
    
    def distance(self, r_id, q1, q2):
        return self.specific_distance(r_id, q1, q2)
    
    def general_distance(self, q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))

    def specific_distance(self, r_id, q1, q2):
        index = self.r_ids.index(r_id)
        model = self.robot_models[index]
        return model.distance_metric(q1, q2)

    #  Collision checking
    def is_collision_free_sample(self, sample, r_id):
        # Check for collisions between the robot and the obstacles
        r_index = self.r_ids.index(r_id)
        self.robot_models[r_index].set_arm_pose(sample)
        for obstacle in self.obstacles:
            if self.env.robot_obstacle_collision(self.robot_models[r_index], obstacle):
                return False
        return True
    
    def is_collision_free_node(self, node, r_id):
        return self.is_collision_free_sample(node.q, r_id)

    def is_collision_free_edge(self, q1, q2, r_id):
        # Check for collisions along the line segment between s and g
        q1 = np.array(q1)
        q2 = np.array(q2)
        for t in np.arange(0, 1, self.local_step):
            sample = q1 + t * (q2 - q1)
            if not self.is_collision_free_sample(sample, r_id):
                return False
        return True
    
    def robot_robot_collision(self, q1, q2, r_1_id, r_2_id):
        r_1_index = self.r_ids.index(r_1_id)
        r_2_index = self.r_ids.index(r_2_id)
        self.robot_models[r_1_index].set_arm_pose(q1)
        self.robot_models[r_2_index].set_arm_pose(q2)
        return self.env.robot_robot_collision(self.robot_models[r_1_index], self.robot_models[r_2_index])

    def transition_valid(self, r_id, d_to_n_1, n_1_id, n_2_id, conflict_times, constraints_from_t):
        r_index = self.r_ids.index(r_id)
        # Get the configurations for the current node and the neighbor node
        q1 = np.array(self.node_id_q_dicts[r_index][n_1_id])
        q2 = np.array(self.node_id_q_dicts[r_index][n_2_id])

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
    
    def robot_robot_collision_on_edge(self, current_n_id, neighbor_n_id, r_id, higher_priority_robot_paths, start_time):
        r_index = self.r_ids.index(r_id)
        # Get the configurations for the current node and the neighbor node
        current_config = self.node_id_q_dicts[r_index][current_n_id]
        neighbor_config = self.node_id_q_dicts[r_index][neighbor_n_id]

        # Calculate the time interval between the current node and the neighbor node
        time_interval = np.linalg.norm(np.array(current_config) - np.array(neighbor_config)) / self.local_step * self.time_step

        # Append the start time to the current and neighbor configurations
        current_config_with_time = np.append(current_config, start_time)
        neighbor_config_with_time = np.append(neighbor_config, start_time + time_interval)

        # Calculate the start and end times for the interval
        interval_start_time = np.round(np.ceil(start_time / self.time_step) * self.time_step, 1)
        interval_end_time = np.round(np.floor((start_time + time_interval) / self.time_step) * self.time_step, 1)

        # Discretize the path from the start configuration to the end configuration
        discretized_path = {}
        for t in np.arange(interval_start_time, interval_end_time + self.time_step, self.time_step):   # np_arange(start, stop) -> [start, stop) 
            q_at_t = self.find_q_in_q_t_interval_given_t(current_config_with_time, neighbor_config_with_time, t)
            discretized_path.update({np.round(t, 1): q_at_t})

        # Check for collisions with higher priority robots
        for other_robot_id in higher_priority_robot_paths:
            other_discretized_path = higher_priority_robot_paths[other_robot_id]
            for t in np.arange(interval_start_time, interval_end_time + self.time_step, self.time_step):
                t = np.round(t, 1)
                config1 = discretized_path[t]
                config2 = other_discretized_path[min(other_discretized_path.keys(), key=lambda x: abs(x - t))]  # this makes sure that for max(time_keys) in other_discretized_path < t the goal config is selected
                if self.robot_robot_collision(config1, config2, r_id, other_robot_id):
                    return True
        return False
    
    def find_q_in_q_t_interval_given_t(self, q1_t1, q2_t2, t):
        delta_t = q2_t2[-1] - q1_t1[-1]
        x = (t - q1_t1[-1]) / delta_t
        return np.round(q1_t1[:-1] + (q2_t2[:-1] - q1_t1[:-1]) * x, 2)

    #  Query methods
    def query(self, priorities=None):
        if priorities is None:
            priorities = [i for i in range(1, len(self.r_ids)+1)]
        assert len(priorities) == len(self.r_ids), "Number of priorities should match the number of robots."
        paths = {}
        order = np.argsort(priorities)[::-1]
        for priority_index in order:
            r_id = self.r_ids[priority_index]
            st_path = self.query_robot(r_id, higher_priority_robot_paths=paths)
            if not st_path:
                return {}
            paths.update({r_id: st_path})
        return paths
    
    def query_robot(self, r_id, higher_priority_robot_paths): 
        r_index = self.r_ids.index(r_id)
        path = []
        start_config = self.agents[r_index]['start']
        goal_config = self.agents[r_index]['goal']

        if not self.is_collision_free_sample(start_config, r_id): 
            print(f"Start configuration for robot {r_id} is in collision.")
            return path
        if not self.is_collision_free_sample(goal_config, r_id):
            print(f"Goal configuration for robot {r_id} is in collision.")
            return path

        # add start and goal to the roadmap
        self.add_start_goal_nodes(r_id)

        # Use Dijkstra to find a path between the nearest start and goal nodes
        # dijkstra_start_time = time.perf_counter()
        # d_path, d_distance = self.dijkstra_search(r_id, higher_priority_robot_paths)
        # dijkstra_duration = time.perf_counter() - dijkstra_start_time
        # print(f"Dijkstra: Composite path in {dijkstra_duration:.6f} seconds with distance {d_distance:.2f}")
        
        # Use A* to find a path between the nearest start and goal nodes
        a_star_start_time = time.perf_counter()
        a_path, a_distance = self.a_star_search(r_id, higher_priority_robot_paths)
        a_star_duration = time.perf_counter() - a_star_start_time
        # print(f"A*: Composite path in {a_star_duration:.6f} seconds with distance {a_distance:.2f}")

        # Store the path for the agent
        path = a_path
        if not path:
            print(f"No path found for robot {r_id}.")
            self.delete_start_goal_nodes(r_id) # Reset roadmap
            return {}
        else: 
            discretized_st_path = self.discretize_path_in_time(r_id, path)
            self.delete_start_goal_nodes(r_id) # Reset roadmap
            return discretized_st_path
        
    def add_start_goal_nodes(self, r_id):
        r_index = self.r_ids.index(r_id)
        start_config = self.agents[r_index]['start']
        goal_config = self.agents[r_index]['goal']

        if not self.is_collision_free_sample(start_config, r_id): 
            print(f"Start configuration for robot {r_id} is in collision.")
            return
        if not self.is_collision_free_sample(goal_config, r_id):
            print(f"Goal configuration for robot {r_id} is in collision.")
            return
        
        # Add start node to roadmap based on distance 
        self.edge_dicts[r_index].update({-1: []})  # -1 -> start config
        for node in self.node_lists[r_index]:
            if len(self.edge_dicts[r_index][-1]) == self.k2:
                break
            if self.distance(r_id, start_config, node.q) <= self.maxdist and self.is_collision_free_edge(start_config, node.q, r_id):
                self.edge_dicts[r_index][-1].append(node.id)
                self.edge_dicts[r_index][node.id].append(-1)
        self.node_id_q_dicts[r_index].update({-1: tuple(start_config)})
        self.node_q_id_dicts[r_index].update({tuple(start_config): -1})

        # Add goal node to roadmap based on distance
        self.edge_dicts[r_index].update({-2: []})  # -2 -> goal config
        for node in self.node_lists[r_index]:
            if len(self.edge_dicts[r_index][-2]) == self.k2:
                break
            if self.distance(r_id, goal_config, node.q) <= self.maxdist and self.is_collision_free_edge(goal_config, node.q, r_id):
                self.edge_dicts[r_index][-2].append(node.id)
                self.edge_dicts[r_index][node.id].append(-2)
        self.node_id_q_dicts[r_index].update({-2: tuple(goal_config)})
        self.node_q_id_dicts[r_index].update({tuple(goal_config): -2})
        return
        
    def delete_start_goal_nodes(self, r_id):
        # Get the index of the given r_id in self.r_ids
        r_index = self.r_ids.index(r_id)
        
        # Remove entries for node -1 and node -2 in edge_dicts
        if -1 in self.edge_dicts[r_index]:
            self.edge_dicts[r_index].pop(-1)
        if -2 in self.edge_dicts[r_index]:
            self.edge_dicts[r_index].pop(-2)
        
        # Remove references to nodes -1 and -2 in neighbor lists of other nodes
        for key in list(self.edge_dicts[r_index].keys()):
            if -1 in self.edge_dicts[r_index][key]:
                self.edge_dicts[r_index][key].remove(-1)
            if -2 in self.edge_dicts[r_index][key]:
                self.edge_dicts[r_index][key].remove(-2)

        # Get the configurations for nodes -1 and -2
        start_config = self.node_id_q_dicts[r_index][-1]
        goal_config = self.node_id_q_dicts[r_index][-2]

        # Remove entries for node -1 and node -2 in node_id_q_dicts
        self.node_id_q_dicts[r_index].pop(-1)
        self.node_id_q_dicts[r_index].pop(-2)

        # Remove entries for the start and goal configurations in node_q_id_dicts
        self.node_q_id_dicts[r_index].pop(start_config)
        self.node_q_id_dicts[r_index].pop(goal_config)
        return

    def discretize_path_in_time(self, r_id, id_path):
        # Calculate index of r_id in self.r_ids
        r_index = self.r_ids.index(r_id)

        # Initialize the discretized path    
        discretized_path = {}

        # Calculate the start and end time of the first interval
        interval_start_t = 0.0
        interval_end_t = 0.0 

        # Discretize the intervals in id_path
        for i in range(len(id_path) - 1):
            interval_start_t = interval_end_t
            start_config = np.array(self.node_id_q_dicts[r_index][id_path[i]])
            end_config = np.array(self.node_id_q_dicts[r_index][id_path[i+1]])
            interval_end_t = interval_start_t + np.linalg.norm(end_config - start_config) / self.local_step * self.time_step
            first_multiple_of_timestep = np.round(np.ceil(interval_start_t / self.time_step) * self.time_step, 1)
            for t in np.arange(first_multiple_of_timestep, interval_end_t, self.time_step):
                t = np.round(t, 1)
                q = self.find_q_in_q_t_interval_given_t(np.append(start_config, interval_start_t), np.append(end_config, interval_end_t), t)
                discretized_path[t] = q

        # Add the goal configuration to the discretized path
        discretized_path[interval_end_t] = end_config

        return discretized_path
    
    def a_star_search(self, r_id, higher_priority_robot_paths):
        """Perform A* search using node IDs."""
        r_index = self.r_ids.index(r_id)

        # Constants
        start_node_id = -1
        goal_node_id = -2
        start_time_interval = 0.0

        # Initialize distances and previous node tracking
        distance_from_start = {node_id: INF for node_id in self.edge_dicts[r_index].keys()}
        distance_from_start[start_node_id] = 0
        estimated_total_cost = {node_id: INF for node_id in self.edge_dicts[r_index].keys()}
        estimated_total_cost[start_node_id] = self.heuristic(start_node_id, goal_node_id, r_id)
        previous_node = {node_id: None for node_id in self.edge_dicts[r_index].keys()}

        unvisited_nodes = set(self.edge_dicts[r_index].keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Choose the unvisited node with the smallest estimated total cost
            current_node_id = min(unvisited_nodes, key=lambda node_id: estimated_total_cost[node_id])
            unvisited_nodes.remove(current_node_id)

            if distance_from_start[current_node_id] == INF:
                break  # Remaining nodes are unreachable

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            t = float(distance_from_start[current_node_id] / self.local_step) * self.time_step + start_time_interval
            for neighbor_id in self.edge_dicts[r_index][current_node_id]:
                if self.robot_robot_collision_on_edge(current_node_id, neighbor_id, r_id, higher_priority_robot_paths, t):
                    continue
                neighbor_config = self.node_id_q_dicts[r_index][neighbor_id]
                current_node_config = self.node_id_q_dicts[r_index][current_node_id]
                # Calculate the distance between current node and neighbor
                distance = self.distance(r_id, neighbor_config, current_node_config)

                new_distance = distance_from_start[current_node_id] + distance
                if new_distance < distance_from_start[neighbor_id]:
                    distance_from_start[neighbor_id] = new_distance
                    estimated_total_cost[neighbor_id] = new_distance + self.heuristic(neighbor_id, goal_node_id, r_id)
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

    def heuristic(self, node_id1, node_id2, r_id):
        # Calculate index of r_id in self.r_ids
        r_index = self.r_ids.index(r_id)
        
        q1 = np.array(self.node_id_q_dicts[r_index][node_id1])
        q2 = np.array(self.node_id_q_dicts[r_index][node_id2])
        return self.distance(r_id, q1, q2)
    
    # def reconstruct_path(self, current_id, came_from):
    #     path = [current_id]
    #     while current_id in came_from.keys():
    #         current_id = came_from[current_id]
    #         path.append(current_id)
    #     return path[::-1]
    
    def compute_total_path_length(self, paths):
        total_length = 0
        for r_id, path in paths.items():
            path_length = 0
            for t, q in path.items():
                if t == 0:
                    continue
                q_prev = path[np.round(t-self.time_step, 1)]
                path_length += self.distance(r_id, q_prev, q)
            total_length += path_length
        return total_length