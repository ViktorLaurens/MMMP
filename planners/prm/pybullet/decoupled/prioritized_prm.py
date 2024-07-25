"""

Python implementation of Prioritized PRM for pybullet simulation

"""
from collections import deque
import time

import numpy as np
from planners.prm.pybullet.decoupled.decoupled_prm import DecoupledPRM
from planners.prm.pybullet.utils import INF

class PrioritizedPRM(DecoupledPRM):
    def __init__(self, environment, maxdist=0, k1=0, k2=0, build_type='kdtree', prm_type='distance', n=0, t=0., time_step=0., local_step=0.) -> None:
        super().__init__(environment, maxdist, k1, k2, build_type, prm_type, n, t, time_step, local_step)
    
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