"""

Decoupled PRM Planner for Pybullet Robot Classes

Notes: 

 - BLUEPRINT: This PRM class cannot be used as a standalone planner BUT
is a blueprint for more specific PRM classes that inherit from it.

 - MULTI ROBOT USE: This PRM class can handle 1 or more robots in a pybullet simulation environment.

"""
import time
import numpy as np

from scipy.spatial import KDTree
from planners.prm.pybullet.utils import Node
from robots.panda import Panda


class DecoupledPRM(): 
    def __init__(self, environment, maxdist=None, k1=20, k2=10, build_type='kdtree', n=100, t=10, time_step=0.1, local_step=None) -> None:
        # unpack environmental data
        self.env = environment
        self.agents = environment.agents                
        self.robot_models = environment.robot_models    
        self.obstacles = environment.obstacles

        # robot handling
        self.r_ids = [environment.robot_models[i].r_id for i in range(len(environment.robot_models))]
        self.c_spaces = self.create_c_spaces(environment.robot_models)

        # Attributes
        self.maxdist = maxdist
        if self.maxdist is None:
            self.maxdist = self.calc_maxdist(self.r_ids, self.c_spaces, 5)

        self.k1 = k1
        self.k2 = k2
        self.build_type = build_type
        self.n = n
        self.t = t

        self.time_step = time_step
        self.local_step = local_step
        if self.local_step is None:
            self.local_step = self.maxdist / 10

        # Create data structures for roadmaps
        self.node_lists = self.create_node_lists(environment.robot_models)
        self.edge_dicts = self.create_edge_dicts(environment.robot_models) # This holds the actual roadmaps, i.e. how the nodes are connected to form a connected graph
        self.node_id_q_dicts = self.create_node_id_q_dicts(environment.robot_models)
        self.node_q_id_dicts = self.create_node_q_id_dicts(environment.robot_models)   

        # Generate roadmaps, i.e. the learning phase
        self.generate_roadmaps()

    #  Initialization methods
    def create_c_spaces(self, robot_models):
        c_spaces = []
        for model in robot_models:
            if isinstance(model, Panda):
                c_spaces.append(model.arm_c_space)
            else: 
                c_spaces.append(model.c_space)
        return c_spaces
    
    def calc_maxdists(self, r_ids, c_spaces, res):
        assert len(r_ids) == len(c_spaces)
        maxdists = {}
        for r_id, c_space in zip(r_ids, c_spaces): 
            unit_vector = np.array([])
            for interval in c_space: 
                unit_vector = np.append(unit_vector, (interval.upper - interval.lower) / res + interval.lower)
            maxdists.update({r_id: self.distance(r_id, unit_vector, [interval.lower for interval in c_space])})
        return maxdists
    
    def calc_res_from_n_and_k2(self, n, k2):
        pass
    
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
                sample = np.random.uniform([interval.lower for interval in self.c_spaces[r_index]], 
                                            [interval.upper for interval in self.c_spaces[r_index]], 
                                            len(self.c_spaces[r_index])
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
                sample = np.random.uniform([interval.lower for interval in self.c_spaces[r_index]], 
                                            [interval.upper for interval in self.c_spaces[r_index]], 
                                            len(self.c_spaces[r_index])
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
            # samples = np.random.uniform([interval.lower for interval in self.c_spaces[r_id]],
            #                             [interval.upper for interval in self.c_spaces[r_id]],
            #                             (self.n, len(self.c_spaces[r_id]))
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
                    # d as given by the query of the kdtree is the minkowski distance, not the specifically defined distance metric
                    d = self.distance(r_id, q, self.node_id_q_dicts[r_index][id2]) # this conversion is needed to compare d to maxdist in following 'if' statement
                    if len(self.edge_dicts[r_index][id]) >= self.k2: 
                        break
                    if d > 0 \
                    and d < self.maxdist \
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
            sample = np.random.uniform([interval.lower for interval in self.c_spaces[r_index]], 
                                        [interval.upper for interval in self.c_spaces[r_index]], 
                                        len(self.c_spaces[r_index])
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

    # Distance
    def distance(self, r_id, q1, q2):
        return self.specific_distance(r_id, q1, q2)
    
    def general_distance(self, q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))

    def specific_distance(self, r_id, q1, q2):
        index = self.r_ids.index(r_id)
        model = self.robot_models[index]
        return model.distance_metric(q1, q2)
    
    # Collision checking
    def is_collision_free_sample(self, sample, r_id):
        # self collisions
        if self.robot_self_collision(sample, r_id):
            return False
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
    
    def robot_self_collision(self, q, r_id):
        r_index = self.r_ids.index(r_id)
        model = self.robot_models[r_index]
        model.set_arm_pose(q)
        return self.env.robot_self_collision(model)
    
    def robot_robot_collision(self, q1, q2, r_1_id, r_2_id):
        r_1_index = self.r_ids.index(r_1_id)
        r_2_index = self.r_ids.index(r_2_id)
        self.robot_models[r_1_index].set_arm_pose(q1)
        self.robot_models[r_2_index].set_arm_pose(q2)
        return self.env.robot_robot_collision(self.robot_models[r_1_index], self.robot_models[r_2_index])

    # Interpolation
    def find_q_in_q_t_interval_given_t(self, q1_t1, q2_t2, t):
        delta_t = q2_t2[-1] - q1_t1[-1]
        x = (t - q1_t1[-1]) / delta_t
        return np.round(q1_t1[:-1] + (q2_t2[:-1] - q1_t1[:-1]) * x, 2)
    
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
    
    # Query
    def query(self): 
        raise NotImplementedError("Query method not implemented.")
    
    def query_robot(self, r_id): 
        raise NotImplementedError("Query method not implemented.")
    
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
    
    def a_star_search(self, r_id): 
        raise NotImplementedError("A* search method not implemented.")
    
    def heuristic(self, n_id1, n_id2, r_id):
        # Calculate index of r_id in self.r_ids
        r_index = self.r_ids.index(r_id)
        
        q1 = np.array(self.node_id_q_dicts[r_index][n_id1])
        q2 = np.array(self.node_id_q_dicts[r_index][n_id2])
        return self.distance(r_id, q1, q2)
    
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

    

    

    