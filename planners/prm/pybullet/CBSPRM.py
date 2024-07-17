"""

Python implementation of Conflict-based Search PRM for pybullet simulation

"""
from itertools import combinations
from copy import deepcopy
import time

import numpy as np
from scipy.spatial import KDTree

from planners.prm.pybullet.utils import Node

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
        # self.robot_ids = np.linspace(0, len(environment.robot_models)-1, len(environment.robot_models), dtype=int)
        self.r_ids = [environment.robot_models[i].r_id for i in range(len(environment.robot_models))]
        self.config_spaces = self.create_config_spaces(environment.robot_models)
        self.maxdist = maxdist
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
            config_spaces.append(model.arm_c_space)
        return config_spaces
    
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
    
    def find_q_in_q_t_interval_given_t(self, q1_t1, q2_t2, t):
        delta_t = q2_t2[-1] - q1_t1[-1]
        x = (t - q1_t1[-1]) / delta_t
        return np.round(q1_t1[:-1] + (q2_t2[:-1] - q1_t1[:-1]) * x, 2)

    #  Query methods
    def query(self):
        start = HighLevelNode()
        # self.n_ct_nodes += 1
        for r_id in self.r_ids:
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
        for r_id in self.r_ids:
            constraints = constraint_dict[r_id]
            id_path = self.query_robot(r_id, constraints) # path is id_path
            if not id_path:
                return {}
            solution.update({r_id: id_path})
        return solution
    
    def compute_cost(self, solution={}):
        total_cost = 0
        for r_id, id_path in solution.items():
            r_index = self.r_ids.index(r_id)
            cost = 0
            for i in range(len(id_path) - 1):
                cost += self.distance(r_id, self.node_id_q_dicts[r_index][id_path[i]], self.node_id_q_dicts[r_index][id_path[i+1]])
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
        
    def delete_start_goal_nodes(self, r_id, start_config, goal_config, start_node, goal_node):
        r_index = self.r_ids.index(r_id)

        self.edge_dicts[r_index].pop(-1)
        self.edge_dicts[r_index].pop(-2)
        self.edge_dicts[r_index][start_node.id].remove(-1)
        self.edge_dicts[r_index][goal_node.id].remove(-2)
        self.node_id_q_dicts[r_index].pop(-1)
        self.node_id_q_dicts[r_index].pop(-2)
        self.node_q_id_dicts[r_index].pop(tuple(start_config))
        self.node_q_id_dicts[r_index].pop(tuple(goal_config))
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

    #  Astar search
    def a_star_search(self, r_id, constraints):
        # Calculate index of r_id in self.r_ids
        r_index = self.r_ids.index(r_id)

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
        g_score = {node_id: np.inf for node_id in self.edge_dicts[r_index].keys()}
        g_score[start_id] = 0
        f_score = {node_id: np.inf for node_id in self.edge_dicts[r_index].keys()}
        f_score[start_id] = self.heuristic(start_id, goal_id, r_id)

        unvisited_nodes = set(self.edge_dicts[r_index].keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Get the node with the lowest f score
            current_id = min(unvisited_nodes, key=lambda node_id: f_score[node_id])
            unvisited_nodes.remove(current_id)

            if g_score[current_id] == np.inf: # Remaining nodes are unreachable
                return [], np.inf # No path found

            if current_id == goal_id: # Check if the goal has been reached
                return self.reconstruct_path(current_id, came_from), g_score[goal_id] # Path found, reconstruct the path from start to goal

            # Check each neighbor of the current node
            for neighbor_id in self.edge_dicts[r_index][current_id]:
                if not self.transition_valid(r_id, g_score[current_id], current_id, neighbor_id, conflict_times, constraints_from_t):
                    continue
                
                current_q = self.node_id_q_dicts[r_index][current_id]
                neighbor_q = self.node_id_q_dicts[r_index][neighbor_id]
                # Calculate the distance between current node and neighbor
                distance = self.distance(r_id, current_q, neighbor_q)

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
        # Calculate index of r_id in self.r_ids
        r_index = self.r_ids.index(r_id)
        
        q1 = np.array(self.node_id_q_dicts[r_index][node_id1])
        q2 = np.array(self.node_id_q_dicts[r_index][node_id2])
        return self.distance(r_id, q1, q2)
    
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
                path_length += self.distance(r_id, q_prev, q)
            total_length += path_length
        return total_length