"""

Distance PRM Planner for Pybullet Robot Classes

Notes: 
 - COUPLED approach
 - MULTI ROBOT USE

"""
import time
from planners.prm.pybullet.coupled.coupled_prm import CoupledPRM
from planners.prm.planar.utils import Node
from scipy.spatial import KDTree

class DistanceCoupledPRM(CoupledPRM): 
    """
    Implementation of Distance PRM Planner for Pybullet Robot Classes
    where each new node is connected to every node within maxdist
    distance that can be connected to it with a collision free straight edge. 
    """
    def __init__(self, environment, max_edge_len, build_type='n', n_nodes=100) -> None:
        super().__init__(environment)
        self.max_edge_len = max_edge_len
        if build_type == 'n':
            n_samples = input("Enter the number of initial nodes: ")
            self.build_roadmap_n(int(n_samples))
        elif build_type == 't':
            build_time = input("Enter the build time: ")
            self.build_roadmap_time(float(build_time))
        elif build_type == 'n_nodes':
            self.build_roadmap_n(n_nodes) 

    # building roadmap
    def build_roadmap_n(self, n):
        return self.add_n_samples(n)
    
    def build_roadmap_time(self, t):
        return self.add_build_period(t)
    
    def add_n_samples(self, n):
        for _ in range(n):
            sample = self.generate_free_sample()
            new_node = Node(id=len(self.nodes), q=sample)
            self.nodes.append(new_node)
            self.roadmap.update({new_node.id: []})
            self.node_q_id_dict.update({new_node.q: new_node.id})
            self.node_id_q_dict.update({new_node.id: new_node.q})
            for node in self.nodes[:-1]:
                if self.distance(new_node.q, node.q) <= self.max_edge_len and self.is_collision_free_edge(new_node.q, node.q):
                    self.roadmap[new_node.id].append(node.id)
                    self.roadmap[node.id].append(new_node.id)
        return
    
    def add_build_period(self, t):
        start_time = time.time()
        while time.time() - start_time < t:
            sample = self.generate_free_sample()
            new_node = Node(id=len(self.nodes), q=sample)
            self.nodes.append(new_node)
            self.roadmap.update({new_node.id: []})
            self.node_q_id_dict.update({new_node.q: new_node.id})
            self.node_id_q_dict.update({new_node.id: new_node.q})
            for node in self.nodes[:-1]:
                if self.distance(new_node.q, node.q) <= self.max_edge_len and self.is_collision_free_edge(new_node.q, node.q):
                    self.roadmap[new_node.id].append(node.id)
                    self.roadmap[node.id].append(new_node.id)
        return
    
    def add_and_update(self, n):
        self.add_n_samples(n)
        return
    
    def update_roadmap(self):
        for i, node in enumerate(self.nodes):
            self.roadmap.update({node.id: []})
            for other_node in self.nodes[i+1:]:
                if self.distance(node.q, other_node.q) <= self.max_edge_len and self.is_collision_free_edge(node.q, other_node.q):
                    self.roadmap[node.id].append(other_node.id)
                    self.roadmap[other_node.id].append(node.id)
        return
    
    # distance metric
    def distance(self, q1, q2):
        return self.general_distance(q1, q2)
    
    # evaluation metrics
    def compute_total_path_length(self, paths):
        total_length = 0
        for agent, path in paths.items():
            total_length += sum([self.distance(path[i], path[i+1]) for i in range(len(path)-1)])
        return total_length
    
class KDTreeDistanceCoupledPRM(CoupledPRM):
    """
    Implementation of Distance PRM Planner for Planar Robots
    in which a KDTree is used for efficient nearest neighbor queries.
    (this requires all the nodes to be sampled first to build the KDTree!)
    """
    def __init__(self, environment, max_edge_len):
        super().__init__(environment)
        self.max_edge_len = max_edge_len

        n_samples = input("Enter the number of initial nodes: ")
        self.build_roadmap_n(int(n_samples))

    # building roadmap
    def build_roadmap_n(self, n):
        self.add_and_update(n)   
        return 
    
    def add_and_update(self, n):
        self.add_n_samples(n)
        self.update_roadmap()
        return
    
    def add_n_samples(self, n):
        for _ in range(n):
            sample = self.generate_free_sample()
            new_node = Node(id=len(self.nodes), q=sample)
            self.nodes.append(new_node)
            self.roadmap.update({new_node.id: []})
            self.node_q_id_dict.update({new_node.q: new_node.id})
            self.node_id_q_dict.update({new_node.id: new_node.q})
        return 
    
    def update_roadmap(self):
        configs = [node.q for node in self.nodes]
        kdtree = KDTree(configs)
        # Populate the roadmap based on max_edge_len
        for i, node in enumerate(self.nodes):
            self.roadmap.update({node.id: []})
            # Query all points within max_edge_len radius
            indices = kdtree.query_ball_point(node.q, r=self.max_edge_len, return_sorted=True)
            for idx in indices[1:]:
                if self.is_collision_free_edge(node.q, configs[idx]): # local planner 
                    # Add to neighbor list
                    self.roadmap[i].append(idx)
        return 

    # distance metric
    def distance(self, q1, q2):
        return self.general_distance(q1, q2)