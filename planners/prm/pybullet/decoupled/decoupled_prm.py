"""

Decoupled PRM Planner for Pybullet Robot Classes

Notes: 

 - BLUEPRINT: This PRM class cannot be used as a standalone planner BUT
is a blueprint for more specific PRM classes that inherit from it.

 - MULTI ROBOT USE: This PRM class can handle 1 or more robots in a pybullet simulation environment.

"""
import heapq
import random
import time
import numpy as np
import pybullet as p

from scipy.spatial import KDTree
from planners.prm.pybullet.utils import Node
from robots.panda import Panda


class DecoupledPRM(): 
    def __init__(self, environment, maxdist=0, k1=0, k2=0, build_type='n', prm_type='degree', n=0, t=0, time_step=0., local_step=0.) -> None:
        # unpack environmental data
        self.env = environment
        self.agents = environment.agents                
        self.robot_models = environment.robot_models    
        self.obstacles = environment.obstacles

        # robot handling
        self.r_ids = [environment.robot_models[i].r_id for i in range(len(environment.robot_models))]
        self.c_spaces = self.create_c_spaces(environment.robot_models)

        # Attributes
        self.maxdist = self.validate_positive_float(maxdist, 'maxdist') if prm_type == 'distance' else maxdist
        # if self.maxdist is None:
        #     self.maxdist = self.calc_maxdist(self.r_ids, self.c_spaces, 5)
        self.k1 = self.validate_positive_integer(k1, 'k1') if prm_type == 'degree' else k1
        self.k2 = self.validate_positive_integer(k2, 'k2') if prm_type == 'degree' else k2
        self.build_type = self.validate_build_type(build_type)
        self.prm_type = self.validate_prm_type(prm_type)
        self.n = self.validate_positive_integer(n, 'n') if build_type in ['n', 'kdtree'] else n
        self.t = self.validate_positive_float(t, 't') if build_type == 't' else t
        self.time_step = self.validate_positive_float(time_step, 'time_step')
        self.local_step = self.validate_positive_float(local_step, 'local_step')
        # if self.local_step is None:
        #     self.local_step = self.maxdist / 10

        # Create data structures for roadmaps
        self.node_lists = self.create_node_lists(environment.robot_models)
        self.edge_dicts = self.create_edge_dicts(environment.robot_models) # This holds the actual roadmaps, i.e. how the nodes are connected to form a connected graph
        self.node_id_q_dicts = self.create_node_id_q_dicts(environment.robot_models)
        self.node_q_id_dicts = self.create_node_q_id_dicts(environment.robot_models)   

        # Generate roadmaps, i.e. the learning phase
        self.generate_roadmaps()
        # update_flag = input('Finetune parameters? (y/N)')
        # if update_flag.lower() == 'y':
        #     update_parameters = True
        # else: 
        #     update_parameters = False
        # while update_parameters:
        #     print(f"\nCurrent parameters: \
        #         \nprm_type = {self.prm_type} \
        #         \nmaxdist = {self.maxdist} \
        #         \nk1 = {self.k1} \
        #         \nk2 = {self.k2} \
        #         \nbuild_type = {self.build_type} \
        #         \nn = {self.n} \
        #         \navg_degree = {self.compute_combined_avg_degree()}")
        #     print("\nOptions: \
        #             \n1 - Change PRM type \
        #             \n2 - Change maxdist \
        #             \n3 - Change k1 \
        #             \n4 - Change k2 \
        #             \n5 - Change build type \
        #             \n6 - Change n \
        #             \n7 - Exit")
        #     choice = input("Enter number: ")
        #     if choice=='1':
        #         new_prm_type = input("Enter new prm_type: ")
        #         self.prm_type = self.validate_prm_type(new_prm_type)
        #         self.update_roadmap_edges()
        #         print(f"Updated prm_type to {new_prm_type}.")
        #     elif choice=='2':
        #         new_maxdist = float(input("Enter new maxdist: "))
        #         self.maxdist = self.validate_positive_float(new_maxdist, 'maxdist')
        #         self.update_roadmaps_edges()
        #         print(f"Updated maxdist to {new_maxdist}.")
        #     elif choice=='3':
        #         new_k1 = int(input("Enter new k1: "))
        #         self.k1 = self.validate_positive_integer(new_k1, 'k1')
        #         self.update_roadmaps_edges()
        #         print(f"Updated k1 to {new_k1}.")
        #     elif choice=='4':
        #         new_k2 = int(input("Enter new k2: "))
        #         self.k2 = self.validate_positive_integer(new_k2, 'k2')
        #         self.update_roadmaps_edges()
        #         print(f"Updated k2 to {new_k2}.")
        #     elif choice=='5':
        #         new_build_type = input("Enter new build_type: ")
        #         self.build_type = self.validate_build_type(new_build_type)
        #         self.update_roadmaps_edges()
        #         print(f"Updated build_type to {new_build_type}.")
        #     elif choice=='6':
        #         prev_n = self.n
        #         new_n = int(input("Enter new n: "))
        #         self.n = self.validate_positive_integer(new_n, 'n')
        #         if prev_n == new_n:
        #             print("n is already set to the new value.")
        #         elif prev_n < new_n:
        #             n_to_add = new_n - prev_n
        #             self.add_nodes_and_update_edges(n_to_add)
        #         elif prev_n > new_n:
        #             n_to_remove = prev_n - new_n
        #             self.remove_nodes_and_update_edges(n_to_remove)
        #         print(f"Updated n to {new_n}.")
        #     elif choice=='7':
        #         print("Exiting parameter update...")
        #         update_parameters = False
        #     else:
        #         print("Invalid option, please enter an integer between 1 and 7.")
        #         continue            

    #  Initialization methods
    def create_c_spaces(self, robot_models):
        c_spaces = []
        for model in robot_models:
            if isinstance(model, Panda):
                c_spaces.append(model.arm_c_space)
            else: 
                c_spaces.append(model.c_space)
        return c_spaces

    @staticmethod
    def validate_positive_integer(value, name):
        assert isinstance(value, int) and value > 0, f"{name} must be a positive integer."
        return value

    @staticmethod
    def validate_positive_float(value, name):
        assert isinstance(value, float) and value > 0, f"{name} must be a positive float."
        return value

    @staticmethod
    def validate_build_type(build_type):
        assert build_type in ['t', 'n', 'kdtree'], "build_type must be 't' or 'n' or 'kdtree'."
        return build_type

    @staticmethod
    def validate_prm_type(prm_type):
        assert prm_type in ['degree', 'distance'], "prm_type must be 'degree' or 'distance'."
        return prm_type
    
    # def calc_maxdists(self, r_ids, c_spaces, res):
    #     assert len(r_ids) == len(c_spaces)
    #     maxdists = {}
    #     for r_id, c_space in zip(r_ids, c_spaces): 
    #         unit_vector = np.array([])
    #         for interval in c_space: 
    #             unit_vector = np.append(unit_vector, (interval.upper - interval.lower) / res + interval.lower)
    #         maxdists.update({r_id: self.distance(r_id, unit_vector, [interval.lower for interval in c_space])})
    #     return maxdists
    
    # def calc_res_from_n_and_k2(self, n, k2):
    #     pass
    
    # def calc_maxdist(self, r_ids, c_spaces, res):
    #     return np.average(list(self.calc_maxdists(r_ids, c_spaces, res).values()))
    
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
        r_index = self.r_ids.index(r_id)

        # build roadmap based on number of nodes
        if self.build_type == 'n':
            # Sample n nodes and allocate memory for roadmap (=edge_dict)
            samples = self.sample_valid_in_task_space(self.n, r_id)
            for i, sample in enumerate(samples):
                new_node = Node(i, sample)
                self.node_lists[r_index].append(new_node)
                self.edge_dicts[r_index].update({new_node.id: []})
            self.node_id_q_dicts[r_index] = self.generate_node_id_q_dict(self.node_lists[r_index])
            self.node_q_id_dicts[r_index] = self.generate_node_q_id_dict(self.node_lists[r_index])
            if self.prm_type == 'degree': 
                self.connect_nearest_neighbors_degree(r_id)
            elif self.prm_type == 'distance': 
                self.connect_nearest_neighbors_distance(r_id)

        # build roadmap based on kdtree, nodes are set a priori, no update of tree possible once created
        elif self.build_type == 'kdtree':
            # Sample n nodes and allocate memory for roadmap (=edge_dict)
            samples = self.sample_valid_in_task_space(self.n, r_id)
            for i, sample in enumerate(samples):
                new_node = Node(i, sample)
                self.node_lists[r_index].append(new_node)
                self.edge_dicts[r_index].update({new_node.id: []})
            self.node_id_q_dicts[r_index] = self.generate_node_id_q_dict(self.node_lists[r_index])
            self.node_q_id_dicts[r_index] = self.generate_node_q_id_dict(self.node_lists[r_index])
            if self.prm_type == 'degree': 
                self.connect_nearest_neighbors_degree_kdtree(r_id)
            elif self.prm_type == 'distance': 
                self.connect_nearest_neighbors_distance_kdtree(r_id)

        # Build roadmap based on time
        elif self.build_type == 't':
            raise NotImplementedError("Update roadmap based on time is not implemented.")
        
        # Sample around start pose
        start_samples = self.sample_start_poses(r_id, n=10)
        start_samples = [Node(len(self.node_lists[r_index]) + i, sample) for i, sample in enumerate(start_samples)]
        self.node_lists[r_index].extend(start_samples)
        self.edge_dicts[r_index].update({node.id: [] for node in start_samples})
        self.node_id_q_dicts[r_index].update(self.generate_node_id_q_dict(start_samples))
        self.node_q_id_dicts[r_index].update(self.generate_node_q_id_dict(start_samples))
        if self.prm_type == 'degree':
            self.connect_extra_nodes_degree(r_id, start_samples)
        elif self.prm_type == 'distance':
            self.connect_extra_nodes_distance(r_id, start_samples)

        # Sample around goal pose
        goal_samples = self.sample_goal_poses(r_id, n=10)
        goal_samples = [Node(len(self.node_lists[r_index]) + i, sample) for i, sample in enumerate(goal_samples)]
        self.node_lists[r_index].extend(goal_samples)
        self.edge_dicts[r_index].update({node.id: [] for node in goal_samples})
        self.node_id_q_dicts[r_index].update(self.generate_node_id_q_dict(goal_samples))
        self.node_q_id_dicts[r_index].update(self.generate_node_q_id_dict(goal_samples))
        if self.prm_type == 'degree':
            self.connect_extra_nodes_degree(r_id, goal_samples)
        elif self.prm_type == 'distance':
            self.connect_extra_nodes_distance(r_id, goal_samples)

        # Sample extra nodes and connect them to the roadmap
        extra_samples = self.sample_safe_poses(r_id)
        extra_nodes = [Node(len(self.node_lists[r_index]) + i, sample) for i, sample in enumerate(extra_samples)]
        self.node_lists[r_index].extend(extra_nodes)
        self.edge_dicts[r_index].update({node.id: [] for node in extra_nodes})
        self.node_id_q_dicts[r_index].update(self.generate_node_id_q_dict(extra_nodes))
        self.node_q_id_dicts[r_index].update(self.generate_node_q_id_dict(extra_nodes))
        if self.prm_type == 'degree':
            self.connect_extra_nodes_degree(r_id, extra_nodes)
        elif self.prm_type == 'distance':
            self.connect_extra_nodes_distance(r_id, extra_nodes)

        # Compute components and try to connect them
        components = self.compute_connected_components(r_id)
        self.connect_connected_components_random(components, r_id, n_pairs=5, max_tries=50)
        return

    # Sampling methods
    def sample_valid_in_joint_space_random(self, n, r_id):
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
    
    def sample_valid_in_task_space(self, n, r_id):
        samples = []
        r_index = self.r_ids.index(r_id)
        panda_model = self.robot_models[r_index]
        start_pos = panda_model.position_from_fk(self.agents[r_index]['start'])
        goal_pos = panda_model.position_from_fk(self.agents[r_index]['goal'])
        # start_quat = panda_model.quaternion_from_fk(self.agents[r_index]['start'])
        start_quat = p.getQuaternionFromEuler([np.radians(180), 0, 0])
        # goal_quat = panda_model.quaternion_from_fk(self.agents[r_index]['goal'])
        goal_quat = p.getQuaternionFromEuler([np.radians(180), 0, 0])
        while len(samples) < n:
            sample_pos = self.sample_task_space_position(start_pos, goal_pos)
            # sample_quat = self.sample_task_space_quaternion(start_quat, goal_quat)
            sample_quat = start_quat
            sample_pose = (sample_pos, sample_quat)
            sample = panda_model.solve_closest_arm_ik(sample_pose, self.agents[r_index]['start'])
            sample = np.round(sample, 2)
            if self.is_collision_free_sample(sample, r_id):
                samples.append(sample)
        # safe_samples = self.sample_safe_poses(r_id)
        # samples.extend(safe_samples)
        return samples

    def sample_task_space_position(self, start_pos, goal_pos, k=1, mu=0.4, sigma=0.2):
        # Convert to numpy arrays if not already
        start_pos = np.array(start_pos)
        goal_pos = np.array(goal_pos)
        # Calculate the unit direction vector between start_pos and goal_pos
        W = goal_pos - start_pos
        W /= np.linalg.norm(W)  # Normalize W
        # Generate orthogonal vectors U and V
        # Choose a fixed orthogonal vector U, avoid U and W being parallel
        if np.allclose(W, [1., 0., 0.]):
            U = np.array([0., 1., 0.])
        else:
            U = np.array([1., 0., 0.])
        U -= np.dot(U, W) * W   # Make U orthogonal to W
        U /= np.linalg.norm(U)  # Normalize U
        V = np.cross(W, U)      # V is orthogonal to both W and U
        # Sample a projection along the line segemnt between start_pos and goal_pos
        t = np.random.uniform(0, 1)
        # Sample a distance away from the projection
        d = mu / k * (sigma**2 / (2*k))**(1/2) * np.random.chisquare(k)
        # Sample an angle around the line segment
        theta = np.random.uniform(0, 2 * np.pi)
        # Calculate and return the new point
        return start_pos + t * (goal_pos-start_pos) + d * (np.cos(theta) * U + np.sin(theta) * V)

    def sample_safe_poses(self, r_id, sigma=0.2):
        r_index = self.r_ids.index(r_id)
        model = self.robot_models[r_index]
        # A selected set of 'safe' poses
        
        # ee central 'safe' poses
        safe_pose1 = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]

        # ee left 'safe' poses
        safe_pose2 = [np.pi/12, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
        safe_pose3 = [np.pi/6, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
        safe_pose4 = [np.pi/4, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]

        # ee right 'safe' poses 
        safe_pose5 = [-np.pi/12, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
        safe_pose6 = [-np.pi/6, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
        safe_pose7 = [-np.pi/4, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]

        # ee high 'safe' poses
        safe_pose8 = [0, -np.pi/4, 0, -np.pi/2, 0, np.pi/4, np.pi/4]
        safe_pose9 = [0, -np.pi/6, 0, -np.pi/2, 0, np.pi/3, np.pi/4]
        safe_pose10 = [0, -np.pi/6, 0, -2*np.pi/3, 0, np.pi/2, np.pi/4]
        safe_pose11 = [0, 0, 0, -np.pi/6, 0, np.pi/6, np.pi/4]
        safe_pose12 = [0, 0, 0, -np.pi/12, 0, np.pi/12, np.pi/4]

        # ee low 'safe' poses
        safe_pose13 = [0, -np.pi/4, 0, -15*np.pi/18, 0, (-np.pi/4) - (-15*np.pi/18), np.pi/4]
        safe_pose14 = [0, -np.pi/6, 0, -15*np.pi/18, 0, (-np.pi/6) - (-15*np.pi/18), np.pi/4]
        safe_pose15 = [0, -np.pi/8, 0, -15*np.pi/18, 0, (-np.pi/8) - (-15*np.pi/18), np.pi/4]
        safe_pose16 = [0, -np.pi/12, 0, -15*np.pi/18, 0, (-np.pi/12) - (-15*np.pi/18), np.pi/4]
        safe_pose17 = [0, 0, 0, -15*np.pi/18, 0, 0 - (-15*np.pi/18), np.pi/4]

        # Additional 'safe' poses
        # Further left 'safe' poses
        safe_pose18 = [np.pi/6, -np.pi/3, 0, -2*np.pi/3, 0, (-np.pi/3) - (-2*np.pi/3), np.pi/4]
        safe_pose19 = [np.pi/8, -np.pi/3, 0, -5*np.pi/6, 0, (-np.pi/3) - (-5*np.pi/6), np.pi/4]

        # Further right 'safe' poses
        safe_pose20 = [-np.pi/6, -np.pi/3, 0, -2*np.pi/3, 0, (-np.pi/3) - (-2*np.pi/3), np.pi/4]
        safe_pose21 = [-np.pi/8, -np.pi/3, 0, -5*np.pi/6, 0, (-np.pi/3) - (-5*np.pi/6), np.pi/4]

        # Low 'safe' poses with different end-effector orientations
        safe_pose22 = [0, -np.pi/4, np.pi/6, -15*np.pi/18, 0, (-np.pi/4) - (-15*np.pi/18), np.pi/4]
        safe_pose23 = [0, -np.pi/6, np.pi/6, -15*np.pi/18, 0, (-np.pi/6) - (-15*np.pi/18), np.pi/4]

        # Extended left 'safe' poses
        safe_pose24 = [np.pi/2, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4]
        safe_pose25 = [np.pi/3, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4]

        # Extended right 'safe' poses
        safe_pose26 = [-np.pi/2, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4]
        safe_pose27 = [-np.pi/3, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4]

        # Extended low 'safe' poses
        safe_pose28 = [0, -np.pi/4, 0, -5*np.pi/6, 0, (-np.pi/4) - (-5*np.pi/6), np.pi/4]
        safe_pose29 = [0, -np.pi/4, 0, -2*np.pi/3, 0, (-np.pi/4) - (-2*np.pi/3), np.pi/4]

        # Combine all poses into a single list
        safe_poses = [
            safe_pose1, safe_pose2, safe_pose3, safe_pose4, safe_pose5, safe_pose6, safe_pose7,
            safe_pose8, safe_pose9, safe_pose10, safe_pose11, safe_pose12, safe_pose13,
            safe_pose14, safe_pose15, safe_pose16, safe_pose17, safe_pose18, safe_pose19,
            safe_pose20, safe_pose21, safe_pose22, safe_pose23, safe_pose24, safe_pose25,
            safe_pose26, safe_pose27, safe_pose28, safe_pose29
        ]

        return safe_poses
    
    def sample_start_poses(self, r_id, n=10):
        r_index = self.r_ids.index(r_id)
        model = self.robot_models[r_index]
        start_pose = self.agents[r_index]['start']
        start_pos = model.position_from_fk(start_pose)
        start_quat = p.getQuaternionFromEuler([np.radians(180), 0, 0])
        start_samples = []
        for _ in range(n):
            start_sample = self.sample_around_task_space_position(start_pos, 0.2)
            start_sample = model.solve_closest_arm_ik((start_sample, start_quat), start_pose)
            start_samples.append(start_sample)
        return start_samples
    
    def sample_goal_poses(self, r_id, n=10):
        r_index = self.r_ids.index(r_id)
        model = self.robot_models[r_index]
        goal_pose = self.agents[r_index]['goal']
        goal_pos = model.position_from_fk(goal_pose)
        goal_quat = p.getQuaternionFromEuler([np.radians(180), 0, 0])
        goal_samples = []
        for _ in range(n):
            goal_sample = self.sample_around_task_space_position(goal_pos, 0.2)
            goal_sample = model.solve_closest_arm_ik((goal_sample, goal_quat), goal_pose)
            goal_samples.append(goal_sample)
        return goal_samples
    
    def sample_around_task_space_position(self, pos, sigma):
        x, y, z = pos
        x += np.random.normal(0, sigma)
        y += np.random.normal(0, sigma)
        z += np.random.normal(0, sigma)
        return [x, y, z]

    # Connection methods
    def heapsort(self, iterable):
        h = []
        for value in iterable:
            heapq.heappush(h, value)
        return [heapq.heappop(h) for _ in range(len(h))]
    
    # # Connect based on degree
    def connect_nearest_neighbors_degree(self, r_id):
        r_index = self.r_ids.index(r_id)
        # Connect nodes with k2 nearest neighbors, constructing the roadmap
        for node in self.node_lists[r_index]:
            # Determine k1 candidate neighbors
            candidate_neighbors = []
            for other_node in self.node_lists[r_index]:
                if node == other_node or self.edge_dicts[r_index][other_node.id] == self.k2:
                    continue
                dist = self.distance(r_id, node.q, other_node.q)
                heapq.heappush(candidate_neighbors, (-dist, other_node))
                if len(candidate_neighbors) > self.k1:
                    heapq.heappop(candidate_neighbors)
            candidate_neighbors = [(-dist, node) for dist, node in candidate_neighbors] # make dists > 0 again
            candidate_neighbors = self.heapsort(candidate_neighbors)
            # Connect the k2 nearest neighbors, that can be connected with collision free edges
            for _, neighbor in candidate_neighbors: # make sure the node itself is excluded
                if len(self.edge_dicts[r_index][node.id]) == self.k2:
                    break
                if self.is_collision_free_edge(node.q, neighbor.q, r_id) \
                    and not neighbor.id in self.edge_dicts[r_index][node.id] \
                    and not len(self.edge_dicts[r_index][neighbor.id]) == self.k2: 
                    self.edge_dicts[r_index][node.id].append(neighbor.id)
                    self.edge_dicts[r_index][neighbor.id].append(node.id)
        return
    
    def connect_nearest_neighbors_degree_kdtree(self, r_id):
        r_index = self.r_ids.index(r_id)
        model = self.robot_models[r_index]
        # Build the KDTree
        samples = [model.position_from_fk(node.q) for node in self.node_lists[r_index]]
        tree = KDTree(samples)
        # Connect nodes with k2 nearest neighbors, constructing the roadmap
        for node in self.node_lists[r_index]:
            # Determine k1 candidate neighbors
            _, candidate_idxs = tree.query([model.position_from_fk(node.q)], k=self.k1+1)[1:] # +1 to exclude the node itself
            for candidate_idx in candidate_idxs:
                if len(self.edge_dicts[r_index][node.id]) == self.k2:
                    break
                neighbor = self.node_lists[r_index][candidate_idx]
                if self.is_collision_free_edge(node.q, neighbor.q, r_id) \
                    and not neighbor.id in self.edge_dicts[r_index][node.id] \
                    and not len(self.edge_dicts[r_index][neighbor.id]) == self.k2: 
                    self.edge_dicts[r_index][node.id].append(neighbor.id)
                    self.edge_dicts[r_index][neighbor.id].append(node.id)
        return
    
    def connect_extra_nodes_degree(self, r_id, nodes):
        r_index = self.r_ids.index(r_id)
        # Connect nodes with k2 nearest neighbors, constructing the roadmap
        for node in nodes:
            # Determine k1 candidate neighbors
            candidate_neighbors = []
            for other_node in self.node_lists[r_index]:
                if node == other_node:
                    continue
                dist = self.distance(r_id, node.q, other_node.q)
                heapq.heappush(candidate_neighbors, (-dist, other_node))
                if len(candidate_neighbors) > self.k1:
                    heapq.heappop(candidate_neighbors)
            candidate_neighbors = [(-dist, node) for dist, node in candidate_neighbors] # make dists > 0 again
            candidate_neighbors = self.heapsort(candidate_neighbors)
            # Connect the k2 nearest neighbors, that can be connected with collision free edges
            for _, neighbor in candidate_neighbors: # make sure the node itself is excluded
                if len(self.edge_dicts[r_index][node.id]) == self.k2:
                    break
                if self.is_collision_free_edge(node.q, neighbor.q, r_id) \
                    and not neighbor.id in self.edge_dicts[r_index][node.id]: 
                    self.edge_dicts[r_index][node.id].append(neighbor.id)
                    self.edge_dicts[r_index][neighbor.id].append(node.id)
        return

    # # Connect based on distance
    def connect_nearest_neighbors_distance(self, r_id):
        r_index = self.r_ids.index(r_id)
        # Connect nodes with k2 nearest neighbors, constructing the roadmap
        for node in self.node_lists[r_index]:
            # Determine candidate neighbors
            candidate_neighbors = []
            for other_node in self.node_lists[r_index]:
                dist = self.distance(r_id, node.q, other_node.q)
                if dist > 0 and dist < self.maxdist: # make sure the node itself is excluded
                    heapq.heappush(candidate_neighbors, (-dist, other_node))
            candidate_neighbors = [(-dist, node) for dist, node in candidate_neighbors] # make dists >0 again
            candidate_neighbors = self.heapsort(candidate_neighbors)
            # Connect the nearest neighbors, that can be connected with collision free edges
            for _, neighbor in candidate_neighbors: 
                if self.is_collision_free_edge(node.q, neighbor.q, r_id) \
                    and not neighbor.id in self.edge_dicts[r_index][node.id]: 
                    self.edge_dicts[r_index][node.id].append(neighbor.id)
                    self.edge_dicts[r_index][neighbor.id].append(node.id)
        return
    
    def connect_nearest_neighbors_distance_kdtree(self, r_id):
        r_index = self.r_ids.index(r_id)
        model = self.robot_models[r_index]
        # Build the KDTree
        samples = [model.position_from_fk(node.q) for node in self.node_lists[r_index]]
        tree = KDTree(samples)
        # Connect nodes with k2 nearest neighbors, constructing the roadmap
        for node in self.node_lists[r_index]:
            # Determine candidate neighbors
            candidate_idxs = tree.query_ball_point([model.position_from_fk(node.q)], self.maxdist)[1:] # +1 to exclude the node itself
            for candidate_idx in candidate_idxs:
                neighbor = self.node_lists[r_index][candidate_idx]
                dist = self.distance(r_id, node.q, neighbor.q)
                if self.is_collision_free_edge(node.q, neighbor.q, r_id) \
                    and not neighbor.id in self.edge_dicts[r_index][node.id]: 
                    self.edge_dicts[r_index][node.id].append(neighbor.id)
                    self.edge_dicts[r_index][neighbor.id].append(node.id)
        return
    
    def connect_extra_nodes_distance(self, r_id, nodes):
        r_index = self.r_ids.index(r_id)
        # Connect nodes with k2 nearest neighbors, constructing the roadmap
        for node in nodes:
            # Determine candidate neighbors
            candidate_neighbors = []
            for other_node in self.node_lists[r_index]:
                dist = self.distance(r_id, node.q, other_node.q)
                if dist > 0 and dist < self.maxdist: # make sure the node itself is excluded
                    heapq.heappush(candidate_neighbors, (-dist, other_node))
            candidate_neighbors = [(-dist, node) for dist, node in candidate_neighbors] # make dists >0 again
            candidate_neighbors = self.heapsort(candidate_neighbors)
            # Connect the nearest neighbors, that can be connected with collision free edges
            for _, neighbor in candidate_neighbors: 
                if self.is_collision_free_edge(node.q, neighbor.q, r_id) \
                    and not neighbor.id in self.edge_dicts[r_index][node.id]: 
                    self.edge_dicts[r_index][node.id].append(neighbor.id)
                    self.edge_dicts[r_index][neighbor.id].append(node.id)
        return
    
    # # Connect components
    def compute_connected_components(self, r_id):
        r_index = self.r_ids.index(r_id)
        visited = set()
        components = []

        def depth_first_search(node):
            component = []
            stack = [node]
            while stack:
                current = stack.pop()
                if current not in visited:
                    visited.add(current)
                    component.append(current)
                    for neighbor_id in self.edge_dicts[r_index][current.id]:
                        neighbor = next(n for n in self.node_lists[r_index] if n.id == neighbor_id)
                        if neighbor not in visited:
                            stack.append(neighbor)
            return component

        for node in self.node_lists[r_index]:
            if node not in visited:
                component = depth_first_search(node)
                components.append(component)
        return components
    
    def connect_connected_components_closest(self, components, r_id):
        r_index = self.r_ids.index(r_id)
        # Iterate through pairs of components
        for i in range(len(components)):
            for j in range(i + 1, len(components)):
                comp1 = components[i]
                comp2 = components[j]
                min_dist = float('inf')
                closest_pair = None
                # Find the closest pair of nodes between the two components
                for node1 in comp1:
                    for node2 in comp2:
                        dist = self.distance(r_id, node1.q, node2.q)
                        if dist < min_dist and self.is_collision_free_edge(node1.q, node2.q, r_id):
                            min_dist = dist
                            closest_pair = (node1, node2)
                if closest_pair:
                    node1, node2 = closest_pair
                    self.edge_dicts[r_index][node1.id].append(node2.id)
                    self.edge_dicts[r_index][node2.id].append(node1.id)

                    # Update components to reflect the new connection
                    comp1.extend(comp2)
                    components[j] = []  # Mark component as empty for removal
        # Remove empty components
        components = [comp for comp in components if comp]
        return components
    
    def connect_connected_components_random(self, components, r_id, n_pairs=5, max_tries=20):
        r_index = self.r_ids.index(r_id)
        # Iterate through pairs of components
        for i in range(len(components)):
            comp1 = components[i]
            if not comp1:
                continue
            for j in range(i + 1, len(components)):
                comp2 = components[j]
                if not comp2:
                    continue
                connecting_pairs = []
                total_pairs = len(comp1) * len(comp2)
                tries = 0
                if total_pairs > n_pairs:
                    # Randomly connect nodes between the two components
                    while len(connecting_pairs) < n_pairs and tries < max_tries:
                        node1 = random.choice(comp1)
                        node2 = random.choice(comp2)
                        if self.is_collision_free_edge(node1.q, node2.q, r_id):
                            connecting_pairs.append((node1, node2))
                        tries += 1
                else: 
                    # Connect all distinct pairs of nodes between the two components
                    for node1 in comp1:
                        for node2 in comp2:
                            if self.is_collision_free_edge(node1.q, node2.q, r_id):
                                connecting_pairs.append((node1, node2))
                # Add edges for the connecting pairs
                for node1, node2 in connecting_pairs:
                    self.edge_dicts[r_index][node1.id].append(node2.id)
                    self.edge_dicts[r_index][node2.id].append(node1.id)
                # Update components to reflect the new connection
                if connecting_pairs:
                    comp1.extend(comp2)
                    components[j] = []  # Mark component as empty for removal
        # Remove empty components
        components = [comp for comp in components if comp]
        return components
    
    # Update roadmap methods
    def update_roadmaps_edges(self):
        for r_id in self.r_ids:
            self.update_roadmap_edges(r_id)
        return
    
    def update_roadmap_edges(self, r_id):
        self.clear_edges(r_id)
        if self.build_type == 'n':
            if self.prm_type == 'degree': 
                self.connect_nearest_neighbors_degree(r_id)
            elif self.prm_type == 'distance': 
                self.connect_nearest_neighbors_distance(r_id)
        elif self.build_type == 'kdtree':
            if self.prm_type == 'degree': 
                self.connect_nearest_neighbors_degree_kdtree(r_id)
            elif self.prm_type == 'distance': 
                self.connect_nearest_neighbors_distance_kdtree(r_id)
        elif self.build_type == 't':
            raise NotImplementedError("Update roadmap based on time is not implemented.")
        components = self.compute_connected_components(r_id)
        self.connect_connected_components_random(components, r_id, n_pairs=5, max_tries=20)
        return
    
    def clear_edges(self, r_id):
        r_index = self.r_ids.index(r_id)
        for node in self.node_lists[r_index]:
            self.edge_dicts[r_index].update({node.id: []})
        return
    
    def add_nodes(self, n):
        for r_id in self.r_ids:
            self.add_nodes_robot(r_id, n)
        return
    
    def add_nodes_robot(self, r_id, n):
        r_index = self.r_ids.index(r_id)
        samples = self.sample_valid_in_task_space(n, r_id)
        samples = [Node(len(self.node_lists[r_index]) + i, sample) for i, sample in enumerate(samples)]
        self.node_lists[r_index].extend(samples)
        self.edge_dicts[r_index].update({node.id: [] for node in samples})
        self.node_id_q_dicts[r_index].update(self.generate_node_id_q_dict(samples))
        self.node_q_id_dicts[r_index].update(self.generate_node_q_id_dict(samples))
        return
    
    def add_nodes_and_update_edges(self, n):
        for r_id in self.r_ids:
            self.add_nodes_robot(r_id, n)
            self.update_roadmap_edges(r_id)
        return
    
    def remove_nodes(self, n):
        for r_id in self.r_ids:
            self.remove_nodes_robot(r_id, n)
        return
    
    def remove_nodes_robot(self, r_id, n):
        r_index = self.r_ids.index(r_id)
        for _ in range(n):
            node = self.node_lists[r_index].pop(-1)
            self.node_id_q_dicts[r_index].pop(node.id)
            self.node_q_id_dicts[r_index].pop(node.q)
        return
    
    def remove_nodes_and_update_edges(self, n):
        for r_id in self.r_ids:
            self.remove_nodes_robot(r_id, n)
            self.update_roadmap_edges(r_id)
        return
    
    # def add_and_update(self, n): 
    #     for r_id in self.r_ids:
    #         self.add_and_update_robot(r_id, n)
    #     return
    
    # def add_and_update_robot(self, r_id, n):
    #     r_index = self.r_ids.index(r_id)
    #     for _ in range(n):
    #         sample = self.sample_valid_in_task_space(1, r_id)[0]
    #         new_node = Node(len(self.node_lists[r_index]), sample)
    #         self.node_lists[r_index].append(new_node)
    #     self.update_roadmap_edges(r_id)
    #     self.node_id_q_dicts[r_index] = self.generate_node_id_q_dict(self.node_lists[r_index])
    #     self.node_q_id_dicts[r_index] = self.generate_node_q_id_dict(self.node_lists[r_index])
    #     return

    # def _find_and_connect_neighbors(self, r_id, new_node):
    #     r_index = self.r_ids.index(r_id)
    #     candidate_neighbors = []
    #     distances = [(self.distance(r_id, new_node.q, node.q), node) 
    #                  for node in self.node_lists[r_index][:-1]]
    #     distances.sort()  # Sort by distance
        
    #     for distance, neighbor in distances:
    #         if distance > self.maxdist or len(candidate_neighbors) == self.k1:
    #             break
    #         if not self._same_component(new_node, neighbor):
    #             candidate_neighbors.append(neighbor)
                
    #     for neighbor in candidate_neighbors:
    #         if len(self.edge_dicts[r_index][new_node.id]) == self.k2:
    #             break
    #         self._try_connect(r_id, new_node, neighbor)
    #     return

    # def _same_component(self, node1, node2):
    #     return False
    
    # def _try_connect(self, r_id, node1, node2):
    #     r_index = self.r_ids.index(r_id)
    #     if self.is_collision_free_edge(node1, node2):
    #         self.edge_dicts[r_index][node1.id].append(node2.id)
    #         self.edge_dicts[r_index][node2.id].append(node1.id)

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
        
        # Add start node to roadmap
        self.edge_dicts[r_index].update({-1: []})  # -1 -> start config
        # Determine k1 candidate neighbors
        candidate_neighbors = []
        for other_node in self.node_lists[r_index]:
            dist = self.distance(r_id, start_config, other_node.q)
            heapq.heappush(candidate_neighbors, (-dist, other_node))
            if len(candidate_neighbors) > self.k1:
                heapq.heappop(candidate_neighbors)
        candidate_neighbors = [(-dist, node) for dist, node in candidate_neighbors] # make dists >0 again
        candidate_neighbors.reverse() # reverse heap which is ordered from furthest to closest
        # Connect the k2 nearest neighbors, that can be connected with collision free edges
        for _, neighbor in candidate_neighbors: # make sure the node itself is excluded
            if len(self.edge_dicts[r_index][-1]) == self.k2:
                break
            if self.is_collision_free_edge(start_config, neighbor.q, r_id):
                self.edge_dicts[r_index][-1].append(neighbor.id)
                self.edge_dicts[r_index][neighbor.id].append(-1)
        self.node_id_q_dicts[r_index].update({-1: tuple(start_config)})
        self.node_q_id_dicts[r_index].update({tuple(start_config): -1})

        # Add goal node to roadmap
        self.edge_dicts[r_index].update({-2: []})  # Just to check the amount of neighbors, not used in query search
        # Determine k1 candidate neighbors
        candidate_neighbors = []
        for other_node in self.node_lists[r_index]:
            dist = self.distance(r_id, goal_config, other_node.q)
            heapq.heappush(candidate_neighbors, (-dist, other_node))
            if len(candidate_neighbors) > self.k1:
                heapq.heappop(candidate_neighbors)
        candidate_neighbors = [(-dist, node) for dist, node in candidate_neighbors] # make dists >0 again
        candidate_neighbors.reverse() # reverse heap which is ordered from furthest to closest
        # Connect the k2 nearest neighbors, that can be connected with collision free edges
        for _, neighbor in candidate_neighbors: # make sure the node itself is excluded
            if len(self.edge_dicts[r_index][-2]) == self.k2:
                break
            if self.is_collision_free_edge(goal_config, neighbor.q, r_id):
                self.edge_dicts[r_index][-2].append(neighbor.id) # needed for nn count
                self.edge_dicts[r_index][neighbor.id].append(-2) # needed for search
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
    
    # compute
    def compute_combined_avg_degree(self):
        total_neighbor_sum = 0
        total_nodes_sum = 0
        for r_id in self.r_ids:
            r_index = self.r_ids.index(r_id)
            total_neighbor_sum += sum([len(neighbors) for neighbors in self.edge_dicts[r_index].values()])
            total_nodes_sum += len(self.node_lists[r_index])
        return total_neighbor_sum / total_nodes_sum
    
    def compute_combined_nr_nodes(self):
        total_nr_nodes = 0
        for r_id in self.r_ids:
            r_index = self.r_ids.index(r_id)
            total_nr_nodes += len(self.node_lists[r_index])
        return total_nr_nodes
    
    def compute_nr_edges(self, r_id):
        r_index = self.r_ids.index(r_id)
        return sum([len(neighbors) for neighbors in self.edge_dicts[r_index].values()]) / 2
    
    def compute_combined_nr_edges(self):
        total_nr_edges = 0
        for r_id in self.r_ids:
            r_index = self.r_ids.index(r_id)
            total_nr_edges += sum([len(neighbors) for neighbors in self.edge_dicts[r_index].values()]) / 2
        return total_nr_edges

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
    


    

    

    