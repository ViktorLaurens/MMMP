"""

CBS PRM Planner for Planar Robots

Notes: 
 - DECOUPLED approach
 - MULTI ROBOT USE

"""
from collections import deque
from copy import deepcopy
from itertools import combinations
import time

import numpy as np
from planners.prm.planar.multi_arm.decoupled.decoupled_prm import DecoupledPRM
from planners.prm.planar.utils import INF, Node


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

# class Constraints:
#     def __init__(self):
#         self.vertex_constraints = set()
#         self.edge_constraints = set()
#     def add_constraint(self, other):
#         self.vertex_constraints |= other.vertex_constraints
#         self.edge_constraints |= other.edge_constraints
#     def __str__(self):
#         return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
#             "EC: " + str([str(ec) for ec in self.edge_constraints])

class ConfigConstraint:
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

class CBSPRM(DecoupledPRM): 
    def __init__(self, environment, max_edge_len, timestep=0.1, build_type='n', n_nodes=100) -> None:
        super().__init__(environment)
        self.max_edge_len = max_edge_len
        self.timestep = timestep
        self.starts = {id: self.agents[id]['start'] for id in self.robot_ids}
        self.goals = {id: self.agents[id]['goal'] for id in self.robot_ids}
        if build_type == 'n':
            n_samples = input("Enter the number of initial nodes per robot: ")
            self.build_roadmap_n(int(n_samples))
        elif build_type == 't':
            build_time = input("Enter the build time per robot: ")
            self.build_roadmap_time(float(build_time))
        elif build_type == 'n_nodes':
            self.build_roadmap_n(n_nodes)

        # evaluation metric
        self.collision_checks = 0
        self.n_ct_nodes = 0

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

    def transition_valid(self, r_id, d_to_n_1, n_1_id, n_2_id, conflict_times, constraints_from_t):
        # Get the configurations for the current node and the neighbor node
        q1 = np.array(self.node_id_config_dict[r_id][n_1_id])
        q2 = np.array(self.node_id_config_dict[r_id][n_2_id])

        # Calculate the time interval between the current node and the neighbor node
        t1 = float(d_to_n_1 / self.local_step) * self.timestep
        t2 = t1 + np.linalg.norm(q2 - q1) / self.local_step * self.timestep
        delta_t = t2 - t1

        # Append the start time to the current and neighbor configurations
        q1_t1 = np.append(q1, t1)
        q2_t2 = np.append(q2, t2)

        # Calculate the start and end times for the interval
        interval_t1 = np.round(np.ceil(t1 / self.timestep) * self.timestep, 1)
        interval_t2 = np.round(np.floor(t2 / self.timestep) * self.timestep, 1)

        # Discretize the path from the start configuration to the end configuration
        discretized_path = {}
        for t in np.round(np.arange(interval_t1, interval_t2 + self.timestep, self.timestep), 1):   # np_arange(start, stop) -> [start, stop) 
            q_at_t = self.find_q_in_q_t_interval_given_t(q1_t1, q2_t2, t)
            discretized_path.update({np.round(t, 1): q_at_t})
            if t in conflict_times: 
                constraints = constraints_from_t[t]
                for constraint in constraints:
                    r_2_q = constraint.q2
                    if self.robot_robot_collision(q_at_t, r_2_q, r_id, constraint.r_2_id):
                        return False

        # # Check if constraints are violated
        # for t in t_to_check:
        #     r_1_q = discretized_path[t]
        #     constraints = constraints_from_t[t]
        #     for constraint in constraints:
        #         r_2_q = constraint.q2
        #         if self.robot_robot_collision(r_1_q, r_2_q, r_id, constraint.r_2_id):
        #             return False
        
        return True

    def find_q_in_q_t_interval_given_t(self, q1_t1, q2_t2, t):
        delta_t = q2_t2[-1] - q1_t1[-1]
        x = (t - q1_t1[-1]) / delta_t
        return np.round(q1_t1[:-1] + (q2_t2[:-1] - q1_t1[:-1]) * x, 2)
        
    # query
    def query(self):
        start = HighLevelNode()
        self.n_ct_nodes += 1
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
                self.n_ct_nodes += 1
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
                cost += self.distance(self.node_id_config_dict[r_id][id_path[i]], self.node_id_config_dict[r_id][id_path[i+1]], r_id)
            total_cost += cost
        return total_cost

    def get_first_conflict_in_time(self, d_solution):
        conflict = Conflict()
        # for r_id, id_path in solution.items():
        #     discretized_solution[r_id] = self.discretize_path_in_time(r_id, id_path)
        max_t = max([max(t_q_path.keys()) for t_q_path in d_solution.values()])
        for t in np.round(np.arange(0, max_t, self.timestep), 1):
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
        constraints[r_id1] = ConfigConstraint(r_id1, r_id2, q2, t)
        constraints[r_id2] = ConfigConstraint(r_id2, r_id1, q1, t)
        return constraints

    def query_robot(self, r_id, constraints={}): 
        path = []
        # start_config = self.agents[r_id]['start']
        # goal_config = self.agents[r_id]['goal']

        # if not self.is_collision_free_sample(start_config, r_id): 
        #     print(f"Start configuration for robot {r_id} is in collision.")
        #     return path
        # if not self.is_collision_free_sample(goal_config, r_id):
        #     print(f"Goal configuration for robot {r_id} is in collision.")
        #     return path
        
        # Find the nearest roadmap nodes to the start and goal configurations
        # start_node = self.find_nearest_roadmap_node(start_config, r_id) # TODO: add start and goal node as any other node based on distance
        # goal_node = self.find_nearest_roadmap_node(goal_config, r_id)

        # add start and goal to the roadmap
        # self.add_start_goal_nodes(r_id, start_config, goal_config, start_node, goal_node)

        # Use Dijkstra to find a path between the nearest start and goal nodes
        # dijkstra_start_time = time.perf_counter()
        # d_path, d_distance = self.dijkstra_search(r_id, constraints)
        # dijkstra_duration = time.perf_counter() - dijkstra_start_time
        # print(f"Dijkstra: Composite path in {dijkstra_duration:.6f} seconds with distance {d_distance:.2f}")
        
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
        self.roadmaps[r_id].update({-1: []})  # -1 -> start config
        for node in self.node_lists[r_id]:
            if self.distance(start_config, node.q, r_id) <= self.max_edge_len and self.is_collision_free_edge(start_config, node.q, r_id):
                self.roadmaps[r_id][-1].append(node.id)
                self.roadmaps[r_id][node.id].append(-1)
        self.node_id_config_dict[r_id].update({-1: tuple(start_config)})
        self.node_config_id_dict[r_id].update({tuple(start_config): -1})

        # Add goal node to roadmap based on distance
        self.roadmaps[r_id].update({-2: []})  # -2 -> goal config
        for node in self.node_lists[r_id]:
            if self.distance(goal_config, node.q, r_id) <= self.max_edge_len and self.is_collision_free_edge(goal_config, node.q, r_id):
                self.roadmaps[r_id][-2].append(node.id)
                self.roadmaps[r_id][node.id].append(-2)
        self.node_id_config_dict[r_id].update({-2: tuple(goal_config)})
        self.node_config_id_dict[r_id].update({tuple(goal_config): -2})
                
        # # Find the nearest roadmap nodes to the start and goal configurations
        # start_node = self.find_nearest_roadmap_node(start_config, r_id) # TODO: add start and goal node as any other node based on distance
        # goal_node = self.find_nearest_roadmap_node(goal_config, r_id)

        # self.roadmaps[r_id].update({-1: [start_node.id]})  # -1 -> start config
        # self.roadmaps[r_id].update({-2: [goal_node.id]})  # -2 -> goal config
        # self.roadmaps[r_id][start_node.id].append(-1)
        # self.roadmaps[r_id][goal_node.id].append(-2)
        # self.node_id_config_dict[r_id].update({-1: tuple(start_config)})
        # self.node_id_config_dict[r_id].update({-2: tuple(goal_config)})
        # self.node_config_id_dict[r_id].update({tuple(start_config): -1})
        # self.node_config_id_dict[r_id].update({tuple(goal_config): -2})
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

    def discretize_path_in_time(self, r_id, id_path):
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
                q = self.find_q_in_q_t_interval_given_t(np.append(start_config, interval_start_t), np.append(end_config, interval_end_t), t)
                discretized_path[t] = q

        # Add the goal configuration to the discretized path
        discretized_path[interval_end_t] = end_config

        return discretized_path
    
    # Search methods
    def dijkstra_search(self, r_id, constraints):
        """Perform Dijkstra's algorithm for r_id using node IDs."""
        # Constants
        start_node_id = -1
        goal_node_id = -2
        # start_time_interval = 0.0

        # compute set of conflict times for more efficient collision checking
        conflict_times = set()
        constraints_from_t = {constraint.t: [] for constraint in constraints}
        for constraint in constraints:
            conflict_times.add(constraint.t)
            constraints_from_t[constraint.t].append(constraint)

        # Initialize distances and previous node tracking
        distance_from_start = {node_id: INF for node_id in self.roadmaps[r_id].keys()}
        distance_from_start[start_node_id] = 0
        previous_node = {node_id: None for node_id in self.roadmaps[r_id].keys()}

        unvisited_nodes = set(self.roadmaps[r_id].keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Choose the unvisited node with the smallest distance
            current_node_id = min(unvisited_nodes, key=lambda node_id: distance_from_start[node_id])
            unvisited_nodes.remove(current_node_id)

            if distance_from_start[current_node_id] == INF:
                break  # Remaining nodes are unreachable

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            for neighbor_id in self.roadmaps[r_id][current_node_id]:
                if not self.transition_valid(r_id, distance_from_start[current_node_id], current_node_id, neighbor_id, conflict_times, constraints_from_t):
                    continue
                neighbor_config = self.node_id_config_dict[r_id][neighbor_id]
                current_node_config = self.node_id_config_dict[r_id][current_node_id]
                # Calculate the distance between current node and neighbor
                distance = self.distance(neighbor_config, current_node_config, r_id)

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
    
    def a_star_search(self, r_id, constraints):
        """Perform A* search using node IDs."""
        # Constants
        start_node_id = -1
        goal_node_id = -2
        # start_time_interval = 0.0

        # compute set of conflict times for more efficient collision checking
        conflict_times = set()
        constraints_from_t = {constraint.t: [] for constraint in constraints}
        for constraint in constraints:
            conflict_times.add(constraint.t)
            constraints_from_t[constraint.t].append(constraint)

        # Initialize distances and previous node tracking
        distance_from_start = {node_id: INF for node_id in self.roadmaps[r_id].keys()}
        distance_from_start[start_node_id] = 0
        estimated_total_cost = {node_id: INF for node_id in self.roadmaps[r_id].keys()}
        estimated_total_cost[start_node_id] = self.heuristic(start_node_id, goal_node_id, r_id)
        previous_node = {node_id: None for node_id in self.roadmaps[r_id].keys()}

        unvisited_nodes = set(self.roadmaps[r_id].keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Choose the unvisited node with the smallest estimated total cost
            current_node_id = min(unvisited_nodes, key=lambda node_id: estimated_total_cost[node_id])
            unvisited_nodes.remove(current_node_id)

            if distance_from_start[current_node_id] == INF:
                break  # Remaining nodes are unreachable

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            for neighbor_id in self.roadmaps[r_id][current_node_id]:
                if not self.transition_valid(r_id, distance_from_start[current_node_id], current_node_id, neighbor_id, conflict_times, constraints_from_t):
                    continue
                neighbor_config = self.node_id_config_dict[r_id][neighbor_id]
                current_node_config = self.node_id_config_dict[r_id][current_node_id]
                # Calculate the distance between current node and neighbor
                distance = self.distance(neighbor_config, current_node_config, r_id)

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

    def heuristic(self, node_id1, node_id2, robot_id):
        """Calculate the distance between two nodes."""
        config1 = np.array(self.node_id_config_dict[robot_id][node_id1])
        config2 = np.array(self.node_id_config_dict[robot_id][node_id2])
        return self.distance(config1, config2, robot_id)

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