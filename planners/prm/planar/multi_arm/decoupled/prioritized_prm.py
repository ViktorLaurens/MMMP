"""

Prioritized PRM Planner for Planar Robots

Notes: 
 - DECOUPLED approach
 - MULTI ROBOT USE

"""
from collections import deque
import time

import numpy as np
from planners.prm.planar.multi_arm.decoupled.decoupled_prm import DecoupledPRM
from planners.prm.planar.utils import INF, Node


class PrioritizedPRM(DecoupledPRM): 
    def __init__(self, environment, max_edge_len, timestep=0.1, build_type='n', n_nodes=100) -> None:
        super().__init__(environment)
        self.max_edge_len = max_edge_len
        self.timestep = timestep
        if build_type == 'n':
            n_samples = input("Enter the number of initial nodes per robot: ")
            self.build_roadmap_n(int(n_samples))
        elif build_type == 't':
            build_time = input("Enter the build time per robot: ")
            self.build_roadmap_time(float(build_time))
        elif build_type == 'n_nodes':
            self.build_roadmap_n(n_nodes)

        # evaluation metrics
        self.collision_checks = 0

    # building roadmap    
    def build_roadmap_n(self, n):
        for id in self.robot_ids: 
            self.add_n_samples(n, id)
        return
    
    def build_roadmap_time(self, t):
        for id in self.robot_ids: 
            self.add_build_period(t, id)
        return
    
    def add_n_samples(self, n, id):
        for _ in range(n):
            sample = self.generate_free_sample(id)
            new_node = Node(id=len(self.node_lists[id]), q=sample)
            self.node_lists[id].append(new_node)
            self.roadmaps[id].update({new_node.id: []})
            self.node_config_id_dict[id].update({new_node.q: new_node.id})
            self.node_id_config_dict[id].update({new_node.id: new_node.q})
            for node in self.node_lists[id][:-1]:
                if self.distance(new_node.q, node.q, id) <= self.max_edge_len and self.is_collision_free_edge(new_node.q, node.q, id):
                    self.roadmaps[id][new_node.id].append(node.id)
                    self.roadmaps[id][node.id].append(new_node.id)
        return

    def add_build_period(self, t, id):
        start_time = time.time()
        while time.time() - start_time < t:
            sample = self.generate_free_sample(id)
            new_node = Node(id=len(self.node_lists[id]), q=sample)
            self.node_lists[id].append(new_node)
            self.roadmaps[id].update({new_node.id: []})
            self.node_config_id_dict[id].update({new_node.q: new_node.id})
            self.node_id_config_dict[id].update({new_node.id: new_node.q})
            for node in self.node_lists[id][:-1]:
                if self.distance(new_node.q, node.q, id) <= self.max_edge_len and self.is_collision_free_edge(new_node.q, node.q, id):
                    self.roadmaps[id][new_node.id].append(node.id)
                    self.roadmaps[id][node.id].append(new_node.id)
        return
    
    def add_and_update(self, n):
        for id in self.robot_ids: 
            self.add_n_samples(n, id)
        return
    
    # def update_roadmap(self): 
    #     for id in self.robot_ids: 
    #         for i, node in enumerate(self.node_lists[id]):
    #             self.roadmaps[id].update({node.id: []})
    #             for other_node in self.node_lists[id][i+1:]:
    #                 if self.distance(node.q, other_node.q, id) <= self.max_edge_len and self.is_collision_free_edge(node.q, other_node.q, id):
    #                     self.roadmaps[id][node.id].append(other_node.id)
    #                     self.roadmaps[id][other_node.id].append(node.id)
    #     return

    def update_roadmap(self): 
        for r_id in self.robot_ids: 
            for node in self.node_lists[r_id]:
                n_1_id = node.id
                self.roadmaps[r_id].update({n_1_id: []})
                for other_node in self.node_lists[r_id]:
                    n_2_id = other_node.id
                    if n_1_id != n_2_id and self.distance(node.q, other_node.q, r_id) <= self.max_edge_len and self.is_collision_free_edge(node.q, other_node.q, r_id):
                        self.roadmaps[r_id][n_1_id].append(n_2_id)
        return

    # distance metric
    def distance(self, q1, q2, robot_id):
        # distance = self.specific_distance(q1, q2, robot_id)
        distance = self.general_distance(q1, q2)
        return distance
    
    # collision checking for query
    def robot_robot_collision(self, config1, config2, robot_id1, robot_id2):
        self.robot_models[robot_id1].set_pose(config1)
        self.robot_models[robot_id2].set_pose(config2)
        return self.env.robot_robot_collision(self.robot_models[robot_id1], self.robot_models[robot_id2])
    
    def robot_robot_collision_on_edge(self, current_node_id, neighbor_node_id, robot_id, higher_priority_robot_paths, start_time):
        # Get the configurations for the current node and the neighbor node
        current_config = self.node_id_config_dict[robot_id][current_node_id]
        neighbor_config = self.node_id_config_dict[robot_id][neighbor_node_id]

        # Calculate the time interval between the current node and the neighbor node
        time_interval = np.linalg.norm(np.array(current_config) - np.array(neighbor_config)) / self.local_step * self.timestep

        # Append the start time to the current and neighbor configurations
        current_config_with_time = np.append(current_config, start_time)
        neighbor_config_with_time = np.append(neighbor_config, start_time + time_interval)

        # Calculate the start and end times for the interval
        interval_start_time = np.round(np.ceil(start_time / self.timestep) * self.timestep, 1)
        interval_end_time = np.round(np.floor((start_time + time_interval) / self.timestep) * self.timestep, 1)

        # Discretize the path from the start configuration to the end configuration
        discretized_path = {}
        for t in np.arange(interval_start_time, interval_end_time + self.timestep, self.timestep):   # np_arange(start, stop) -> [start, stop) 
            q_at_t = self.find_point_in_interval(current_config_with_time, neighbor_config_with_time, t)
            discretized_path.update({np.round(t, 1): q_at_t})

        # Check for collisions with higher priority robots
        for other_robot_id in higher_priority_robot_paths:
            other_discretized_path = higher_priority_robot_paths[other_robot_id]
            for t in np.arange(interval_start_time, interval_end_time + self.timestep, self.timestep):
                t = np.round(t, 1)
                config1 = discretized_path[t]
                config2 = other_discretized_path[min(other_discretized_path.keys(), key=lambda x: abs(x - t))]  # this makes sure that for max(time_keys) in other_discretized_path < t the goal config is selected
                if self.robot_robot_collision(config1, config2, robot_id, other_robot_id):
                    return True
        return False

    def find_point_in_interval(self, config_t1, config_t2, t):
        time_interval = config_t2[-1] - config_t1[-1]
        x = (t - config_t1[-1]) / time_interval
        return np.round(config_t1[:-1] + (config_t2[:-1] - config_t1[:-1]) * x, 2)
        
    # query
    def query(self, priorities=None):
        if priorities is None:
            priorities = [i for i in range(1, len(self.robot_ids)+1)]
        assert len(priorities) == len(self.robot_ids), "Number of priorities should match the number of robots."
        paths = {}
        order = np.argsort(priorities)[::-1]
        for id in order:
            st_path = self.query_robot(id, higher_priority_robot_paths=paths)
            if not st_path:
                return {}
            paths.update({id: st_path})
        return paths

    def query_robot(self, r_id, higher_priority_robot_paths): 
        path = []
        start_config = self.agents[r_id]['start']
        goal_config = self.agents[r_id]['goal']

        if not self.is_collision_free_sample(start_config, r_id): 
            print(f"Start configuration for robot {r_id} is in collision.")
            return path
        if not self.is_collision_free_sample(goal_config, r_id):
            print(f"Goal configuration for robot {r_id} is in collision.")
            return path
        
        # Find the nearest roadmap nodes to the start and goal configurations
        start_node = self.find_nearest_roadmap_node(start_config, r_id) # TODO: add start and goal node as any other node based on distance
        goal_node = self.find_nearest_roadmap_node(goal_config, r_id)

        # add start and goal to the roadmap
        self.add_start_goal_nodes(r_id, start_config, goal_config, start_node, goal_node)

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
            self.delete_start_goal_nodes(r_id, start_config, goal_config, start_node, goal_node) # Reset roadmap
            return {}
        else: 
            discretized_st_path = self.discretize_path(r_id, path)
            self.delete_start_goal_nodes(r_id, start_config, goal_config, start_node, goal_node) # Reset roadmap
            return discretized_st_path
        
    def add_start_goal_nodes(self, r_id, start_config, goal_config, start_node, goal_node):
        self.roadmaps[r_id].update({-1: [start_node.id]})  # -1 -> start config
        self.roadmaps[r_id].update({-2: [goal_node.id]})  # -2 -> goal config
        self.roadmaps[r_id][start_node.id].append(-1)
        self.roadmaps[r_id][goal_node.id].append(-2)
        self.node_id_config_dict[r_id].update({-1: tuple(start_config)})
        self.node_id_config_dict[r_id].update({-2: tuple(goal_config)})
        self.node_config_id_dict[r_id].update({tuple(start_config): -1})
        self.node_config_id_dict[r_id].update({tuple(goal_config): -2})
        return
        
    def delete_start_goal_nodes(self, r_id, start_config, goal_config, start_node, goal_node):
        self.roadmaps[r_id].pop(-1)
        self.roadmaps[r_id].pop(-2)
        self.roadmaps[r_id][start_node.id].remove(-1)
        self.roadmaps[r_id][goal_node.id].remove(-2)
        self.node_id_config_dict[r_id].pop(-1)
        self.node_id_config_dict[r_id].pop(-2)
        self.node_config_id_dict[r_id].pop(tuple(start_config))
        self.node_config_id_dict[r_id].pop(tuple(goal_config))
        return

    def discretize_path(self, r_id, id_path):
        # Initialize the discretized path    
        discretized_path = {}

        # Calculate the start and end time of the first interval
        interval_start_t = 0.0
        interval_end_t = 0.0 

        # Discretize the intervals in id_path
        for i in range(len(id_path) - 1):
            interval_start_t = interval_end_t
            start_config = np.array(self.node_id_config_dict[r_id][id_path[i]])
            end_config = np.array(self.node_id_config_dict[r_id][id_path[i+1]])
            interval_end_t = interval_start_t + np.linalg.norm(end_config - start_config) / self.local_step * self.timestep
            first_multiple_of_timestep = np.round(np.ceil(interval_start_t / self.timestep) * self.timestep, 1)
            for t in np.arange(first_multiple_of_timestep, interval_end_t, self.timestep):
                t = np.round(t, 1)
                q = self.find_point_in_interval(np.append(start_config, interval_start_t), np.append(end_config, interval_end_t), t)
                discretized_path[t] = q

        # Add the goal configuration to the discretized path
        discretized_path[interval_end_t] = end_config

        return discretized_path
    
    # Search methods
    def dijkstra_search(self, robot_id, higher_priority_robot_paths):
        """Perform Dijkstra's algorithm for robot_id using node IDs."""
        # Constants
        start_node_id = -1
        goal_node_id = -2
        start_time_interval = 0.0

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
            t = float(distance_from_start[current_node_id] / self.local_step) * self.timestep + start_time_interval
            for neighbor_id in self.roadmaps[robot_id][current_node_id]:
                if self.robot_robot_collision_on_edge(current_node_id, neighbor_id, robot_id, higher_priority_robot_paths, t):
                    continue
                neighbor_config = self.node_id_config_dict[robot_id][neighbor_id]
                current_node_config = self.node_id_config_dict[robot_id][current_node_id]
                # Calculate the distance between current node and neighbor
                distance = self.distance(neighbor_config, current_node_config, robot_id)

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
    
    def a_star_search(self, robot_id, higher_priority_robot_paths):
        """Perform A* search using node IDs."""
        # Constants
        start_node_id = -1
        goal_node_id = -2
        start_time_interval = 0.0

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

            if distance_from_start[current_node_id] == INF:
                break  # Remaining nodes are unreachable

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            t = float(distance_from_start[current_node_id] / self.local_step) * self.timestep + start_time_interval
            for neighbor_id in self.roadmaps[robot_id][current_node_id]:
                if self.robot_robot_collision_on_edge(current_node_id, neighbor_id, robot_id, higher_priority_robot_paths, t):
                    continue
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

    # metrics
    def compute_total_path_length(self, paths):
        total_length = 0
        for r_id, path in paths.items():
            path_length = 0
            for t, q in path.items():
                if t == 0:
                    continue
                q_prev = path[np.round(t-self.timestep, 1)]
                path_length += self.distance(q_prev, q, r_id)
            total_length += path_length
        return total_length