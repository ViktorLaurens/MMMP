"""

Degree PRM Planner for Planar Robots

Notes: 

 - SINGLE ROBOT USE: This PRM class expects only 1 agent/robot in the environment.

"""
import time
from planners.prm.planar.env import LOCAL_STEP
from planners.prm.planar.single_arm.prm import PRM
from planners.prm.planar.utils import Node
from scipy.spatial import KDTree

class KNNPRM(PRM): # Needs adjustments, logic is not right: rewiring needed
    """
    Implementation of Degree PRM Planner for Planar Robots
    (KNNPRM stands for K-Nearest Neighbors PRM)
    where each new node is connected to its k nearest neighboring nodes 
    that can be connected to it with a collision free straight edge. 
    """
    def __init__(self, environment, max_edge_len, n_knn, build_type='n'):
        super().__init__(environment, local_step=LOCAL_STEP)
        self.max_edge_len = max_edge_len
        self.n_knn = n_knn
        if build_type == 'n':
            n_samples = input("Enter the number of initial nodes: ")
            self.build_roadmap_n(int(n_samples))
        elif build_type == 't':
            build_time = input("Enter the build time: ")
            self.build_roadmap_time(float(build_time))

    # building roadmap
    def build_roadmap_n(self, n):
        return self.add_n_samples(n)

    def build_roadmap_time(self, t):
        return self.add_build_period(t)
    
    def add_and_update(self, n):
        self.add_n_samples(n)
        return
    
    def add_n_samples(self, n):
        for _ in range(n):
            sample = self.generate_free_sample()
            new_node = Node(id=len(self.nodes), q=sample)
            self.nodes.append(new_node)
            self.roadmap.update({new_node.id: []})
            self.node_config_id_dict.update({new_node.q: new_node.id})
            self.node_id_config_dict.update({new_node.id: new_node.q})
            # Find the k-nearest neighbors
            evaluated_nodes = set()
            while len(evaluated_nodes) < len(self.nodes[:-1]):
                if len(self.roadmap[new_node.id]) >= self.n_knn:
                    break
                closest_node = None
                closest_dist = float('inf')
                for node in self.nodes[:-1]:
                    dist = self.distance(new_node.q, node.q)
                    if dist < closest_dist and node not in evaluated_nodes:
                        closest_node = node
                        closest_dist = dist
                if closest_node is not None:
                    evaluated_nodes.add(closest_node)
                    if self.is_collision_free_edge(new_node.q, closest_node.q):
                        self.roadmap[new_node.id].append(closest_node.id)
                        self.roadmap[closest_node.id].append(new_node.id)
        return
        
    def add_build_period(self, t):
        start_time = time.time()
        while time.time() - start_time < t:
            sample = self.generate_free_sample()
            new_node = Node(id=len(self.nodes), q=sample)
            self.nodes.append(new_node)
            self.roadmap.update({new_node.id: []})
            self.node_config_id_dict.update({new_node.q: new_node.id})
            self.node_id_config_dict.update({new_node.id: new_node.q})
            # Find the k-nearest neighbors
            evaluated_nodes = set()
            while len(evaluated_nodes) < len(self.nodes):
                if len(self.roadmap[new_node.id]) >= self.n_knn:
                    break
                closest_node = None
                closest_dist = float('inf')
                for node in self.nodes[:-1]:
                    dist = self.distance(new_node.q, node.q)
                    if dist < closest_dist and node not in evaluated_nodes:
                        closest_node = node
                        closest_dist = dist
                if closest_node is not None:
                    evaluated_nodes.add(closest_node)
                    if self.is_collision_free_edge(new_node.q, closest_node.q):
                        self.roadmap[new_node.id].append(closest_node.id)
                        self.roadmap[closest_node.id].append(new_node.id)
        return 
    
    def update_roadmap(self):
        for i, node in enumerate(self.nodes):
            self.roadmap.update({node.id: []})
            evaluated_nodes = set()
            while len(evaluated_nodes) < len(self.nodes[i+1:]):
                if len(self.roadmap[node.id]) >= self.n_knn:
                    break
                closest_node = None
                closest_dist = float('inf')
                for other_node in self.nodes[i+1:]:
                    dist = self.distance(node.q, other_node.q)
                    if dist < closest_dist and other_node not in evaluated_nodes:
                        closest_node = other_node
                        closest_dist = dist
                if closest_node is not None:
                    evaluated_nodes.add(closest_node)
                    if self.is_collision_free_edge(node.q, closest_node.q):
                        self.roadmap[node.id].append(closest_node.id)
                        self.roadmap[closest_node.id].append(node.id)
        return

    # distance metric
    def distance(self, q1, q2):
        return self.specific_distance(q1, q2)

class KDTreeKNNPRM(PRM):
    """
    Implementation of Degree PRM Planner for Planar Robots
    in which a KDTree is used for efficient nearest neighbor queries.
    (this requires all the nodes to be sampled first to build the KDTree!)
    """ 
    def __init__(self, environment, max_edge_len, n_knn):
        super().__init__(environment, local_step=LOCAL_STEP)
        self.max_edge_len = max_edge_len
        self.n_knn = n_knn

        n_samples = input("Enter the number of initial nodes: ")
        self.build_roadmap_n(int(n_samples))

        # print data
        nr_nodes = self.compute_nr_nodes
        nr_edges = self.compute_nr_edges
        avg_degree = self.compute_avg_degree
        print(f"\nRoadmap data: \nnodes: {nr_nodes} \nedges: {nr_edges} \naverage degree: {avg_degree}")

    # building roadmap
    def build_roadmap_n(self, n):
        self.add_n_samples(n)
        self.update_roadmap()   
        return 

    def add_and_update(self, n): 
        self.add_n_samples(n)
        self.update_roadmap()
        return
        
    def add_n_samples(self, n):
        # new_nodes = []
        for _ in range(n):
            sample = self.generate_free_sample()
            new_node = Node(id=len(self.nodes), q=sample)
            # new_nodes.append(new_node)
            self.nodes.append(new_node)
            self.roadmap.update({new_node.id: []})
            self.node_config_id_dict.update({new_node.q: new_node.id})
            self.node_id_config_dict.update({new_node.id: new_node.q})
        return #new_nodes
    
    def update_roadmap(self):
        return self.update_roadmap_strict_knn()

    def update_roadmap_strict_knn(self): # strictly evaluate knn, collision -> nn not added (knn might be less than k), nn for other node -> nn added (knn might be more than k)
        configs = [node.q for node in self.nodes]
        kdtree = KDTree(configs)
        self.reset_roadmap()
        # Populate the roadmap based on n_knn
        for i, node in enumerate(self.nodes):
            # Check k nearest neighbors for connection
            dists, indices = kdtree.query(node.q, k=self.n_knn+1) # plus 1 because the first index is the node itself
            for idx in indices[1:]:
                if self.is_collision_free_edge(node.q, configs[idx]): # n_knn are evaluated
                    if idx not in self.roadmap[i]:  # Add to neighbor list if not already connected 
                        self.roadmap[i].append(idx)
                        self.roadmap[idx].append(i) # roadmap is undirectional
        return 

    def update_roadmap_min_knn(self): # try to have min k nn
        configs = [node.q for node in self.nodes]
        kdtree = KDTree(configs)
        self.reset_roadmap()
        # Populate the roadmap based on n_knn
        for i, node in enumerate(self.nodes):
            # Query all points within max_edge_len radius
            indices = kdtree.query_ball_point(node.q, r=self.max_edge_len, return_sorted=True)
            for idx in indices[1:]:
                if len(self.roadmap[i]) >= self.n_knn: 
                    break
                elif self.is_collision_free_edge(node.q, configs[idx]) and idx not in self.roadmap[i]: # local planner 
                    # Add to neighbor list
                    self.roadmap[i].append(idx)
                    self.roadmap[idx].append(i) # roadmap is undirectional
        return
    
    def reset_roadmap(self):
        for node in self.nodes:
            self.roadmap[node.id] = []
        return
    
    # distance metric
    def distance(self, q1, q2):
        return self.general_distance(q1, q2)
    
