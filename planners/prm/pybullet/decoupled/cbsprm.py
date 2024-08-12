"""

Python implementation of Conflict-based Search PRM for pybullet simulation

"""
from itertools import combinations
from copy import deepcopy
import time

import numpy as np
# from scipy.spatial import KDTree

from planners.prm.pybullet.decoupled.decoupled_prm import DecoupledPRM
# from planners.prm.pybullet.utils import Node

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

class CBSPRM(DecoupledPRM):
    def __init__(self, environment, load_roadmap, maxdist, k1=0, k2=0, build_type='n', prm_type='degree', n=0, t=0, time_step=0., local_step=0.) -> None:
        super().__init__(environment, load_roadmap, maxdist, k1, k2, build_type, prm_type, n, t, time_step, local_step)
        self.n_ct_nodes = 0

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

    #  Query methods
    def query(self):
        start = HighLevelNode()
        self.n_ct_nodes += 1
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
                # for r_id in self.r_ids:
                #     self.delete_start_goal_nodes(r_id)
                return P.solution
            
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
    
    def reconstruct_path(self, current_id, came_from):
        path = [current_id]
        while current_id in came_from.keys():
            current_id = came_from[current_id]
            path.append(current_id)
        return path[::-1]