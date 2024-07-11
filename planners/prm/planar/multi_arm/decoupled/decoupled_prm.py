"""

Decoupled PRM Planner for Planar Robots

Notes: 

 - BLUEPRINT: This PRM class cannot be used as a standalone planner BUT
is a blueprint for more specific PRM classes that inherit from it.

 - MULTI ROBOT USE: This PRM class can handle 1 or more robots in a single planar environment.

 """
from collections import deque
from matplotlib import pyplot as plt
import numpy as np
from planners.prm.planar.env import LOCAL_STEP
from planners.prm.planar.single_arm.prm import PRM
from planners.prm.planar.utils import INF, Node


class DecoupledPRM(PRM): 
    def __init__(self, environment) -> None:
        super().__init__(environment, local_step=LOCAL_STEP)
        self.robot_ids = np.linspace(0, len(environment.robot_models)-1, len(environment.robot_models), dtype=int)
        self.config_spaces = self.create_config_spaces(environment.robot_models)
        self.roadmaps = self.create_roadmaps(environment.robot_models)
        self.node_lists = self.create_node_lists(environment.robot_models)
        self.node_id_config_dict = self.create_node_id_config_dict(environment.robot_models)
        self.node_config_id_dict = self.create_node_config_id_dict(environment.robot_models)

    # methods for decoupled PRM
    def create_config_spaces(self, robot_models):
        config_spaces = []
        for model in robot_models:
            config_spaces.append(model.config_space)
        return config_spaces

    def create_roadmaps(self, robot_models):
        roadmaps = []
        for _ in robot_models:
            roadmaps.append({})
        return roadmaps
    
    def create_node_lists(self, robot_models):
        node_lists = []
        for _ in robot_models:
            node_lists.append([])
        return node_lists
    
    def create_node_id_config_dict(self, robot_models):
        node_id_config_dict = []
        for _ in robot_models:
            node_id_config_dict.append({})
        return node_id_config_dict
    
    def create_node_config_id_dict(self, robot_models):
        node_config_id_dict = []
        for _ in robot_models:
            node_config_id_dict.append({})
        return node_config_id_dict
    
    # # Overwriting PRM methods to adapt to decoupled PRM
    # dictionaries for mapping id's and configurations
    def update_node_id_config_dict(self, node_lists):
        for i, node_list in enumerate(node_lists):
            for node in node_list:
                self.node_id_config_dict[i].update({node.id: node.q})
        return
    
    def update_node_config_id_dict(self, node_lists):
        for i, node_list in enumerate(node_lists):
            for node in node_list:
                self.node_config_id_dict[i].update({node.q: node.id})
        return
    
    # sampling functions
    def generate_sample(self, config_space):
        sample = np.random.uniform(
            [interval.lower for interval in config_space],
            [interval.upper for interval in config_space],
        )
        sample = np.round(sample, 2)
        return sample
    
    def generate_free_sample(self, robot_id):
        sample = self.generate_sample(self.config_spaces[robot_id])
        while not self.is_collision_free_sample(sample, robot_id):
            sample = self.generate_sample(self.config_spaces[robot_id])
        return sample
    
    def generate_free_samples(self, n, robot_id): 
        # Sample n free nodes
        sampled_free_nodes = []
        for _ in range(n):
            sample = self.generate_free_sample(robot_id)
            node = Node(id=len(self.node_lists[robot_id])+len(sampled_free_nodes), q=sample)
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
    def distance(self, q1, q2, robot_id):
        raise NotImplementedError()
    
    def specific_distance(self, q1, q2, robot_id):
        pos1 = self.robot_models[robot_id].forward_kinematics(q1)
        pos2 = self.robot_models[robot_id].forward_kinematics(q2)
        return self.robot_models[robot_id].distance(pos1, pos2)        
    
    # local planner and collision checking
    def is_collision_free_sample(self, sample, robot_id):
        robot = self.robot_models[robot_id]
        # within map bounds
        if not self.robot_in_env_bounds(sample, robot):
            return False
        # self collisions
        # if self.robot_self_collision(sample, robot):
        #     return False
        # collisions with obstacles
        for obstacle in self.obstacles:
            # sample1 = sample[i*robot1.n_links:(i+1)*robot1.n_links]
            if self.robot_obstacle_collision(sample, robot, obstacle):
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
    
    # Querying roadmap
    def find_nearest_roadmap_node(self, config, robot_id):
        """Finds the nearest roadmap node to a given configuration by looping over all nodes."""
        min_dist = float('inf')
        nearest_node = None
        for node in self.node_lists[robot_id]:
            dist = self.distance(config, node.q, robot_id)
            if dist < min_dist and self.is_collision_free_edge(config, node.q, robot_id):
                min_dist = dist
                nearest_node = node
        return nearest_node

    def query(self): 
        raise NotImplementedError()
    
    # Search methods
    def dijkstra_search(self, start_node_id, goal_node_id, robot_id):
        """Perform Dijkstra's algorithm for robot_id using node IDs."""
        # Initialize distances and previous node tracking
        distance_from_start = {node_id: INF for node_id in self.roadmaps[robot_id].keys()}
        distance_from_start[start_node_id] = 0
        previous_node = {node_id: None for node_id in self.roadmaps[robot_id].keys()}

        unvisited_nodes = set(self.roadmaps[robot_id].keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Choose the unvisited node with the smallest distance
            current_node_id = min(unvisited_nodes, key=lambda node_id: distance_from_start[node_id])
            unvisited_nodes.remove(current_node_id)

            if distance_from_start[current_node_id] == INF:
                break  # Remaining nodes are unreachable

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            for neighbor_id in self.roadmaps[robot_id][current_node_id]:
                neighbor_config = self.node_id_config_dict[robot_id][neighbor_id]
                current_node_config = self.node_id_config_dict[robot_id][current_node_id]
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
    
    def a_star_search(self, start_node_id, goal_node_id, robot_id):
        """Perform A* search using node IDs."""
        # Initialize distances and previous node tracking
        distance_from_start = {node_id: INF for node_id in self.roadmaps[robot_id].keys()}
        distance_from_start[start_node_id] = 0
        estimated_total_cost = {node_id: INF for node_id in self.roadmaps[robot_id].keys()}
        estimated_total_cost[start_node_id] = self.heuristic(start_node_id, goal_node_id, robot_id)
        previous_node = {node_id: None for node_id in self.roadmaps[robot_id].keys()}

        unvisited_nodes = set(self.roadmaps[robot_id].keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Choose the unvisited node with the smallest estimated total cost
            current_node_id = min(unvisited_nodes, key=lambda node_id: estimated_total_cost[node_id])
            unvisited_nodes.remove(current_node_id)

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            for neighbor_id in self.roadmaps[robot_id][current_node_id]:
                neighbor_config = self.node_id_config_dict[robot_id][neighbor_id]
                current_node_config = self.node_id_config_dict[robot_id][current_node_id]
                # Calculate the distance between current node and neighbor
                distance = self.distance(neighbor_config, current_node_config, robot_id)

                new_distance = distance_from_start[current_node_id] + distance
                if new_distance < distance_from_start[neighbor_id]:
                    distance_from_start[neighbor_id] = new_distance
                    estimated_total_cost[neighbor_id] = new_distance + self.heuristic(neighbor_id, goal_node_id, robot_id)
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

    def heuristic(self, node_id1, node_id2, robot_id):
        """Calculate the distance between two nodes."""
        config1 = np.array(self.node_id_config_dict[robot_id][node_id1])
        config2 = np.array(self.node_id_config_dict[robot_id][node_id2])
        return self.distance(config1, config2, robot_id)
    
    # compute
    def compute_avg_degree(self, robot_id):
        print(len(self.roadmaps[robot_id]))
        return sum([len(neighbors) for neighbors in self.roadmaps[robot_id].values()]) / len(self.roadmaps[robot_id])
    
    def compute_combined_avg_degree(self):
        neighbor_sum = 0
        nodes_sum = 0
        for id in self.robot_ids:
            neighbor_sum += sum([len(neighbors) for neighbors in self.roadmaps[id].values()])
            nodes_sum += len(self.roadmaps[id])
        return neighbor_sum / nodes_sum
    
    def compute_nr_nodes(self, robot_id):
        return len(self.node_lists[robot_id])
    
    def compute_combined_nr_nodes(self):
        return sum([len(self.node_lists[id]) for id in self.robot_ids])
    
    def compute_nr_edges(self, robot_id):
        return sum([len(neighbors) for neighbors in self.roadmaps[robot_id].values()]) / 2
    
    def compute_combined_nr_edges(self):
        return sum([sum([len(neighbors) for neighbors in self.roadmaps[id].values()]) for id in self.robot_ids]) / 2
    
    # Plot roadmap
    def plot_roadmap_2D_subspace(self, robot_id):
        fig, ax = plt.subplots() 

        for node, neighbors in self.roadmaps[robot_id].items():
            node_q = self.node_id_config_dict[robot_id][node]
            for neighbor in neighbors:
                neighbor_q = self.node_id_config_dict[robot_id][neighbor]
                ax.plot([node_q[0], neighbor_q[0]], [node_q[1], neighbor_q[1]], 'k-', linewidth=0.5)
        
        for node in self.node_lists[robot_id]:
            ax.plot(node.q[0], node.q[1], 'o', color='blue', markersize=3)

        start_q = self.agents[robot_id]['start']
        # print(start_q)
        goal_q = self.agents[robot_id]['goal']
        # print(goal_q)
        ax.plot(start_q[0], start_q[1], 'o', color='red', markersize=4)
        ax.plot(goal_q[0], goal_q[1], 'o', color='green', markersize=4)
        
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim([self.config_space[0].lower, self.config_space[0].upper])
        ax.set_ylim([self.config_space[1].lower, self.config_space[1].upper])
        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks
        plt.show()