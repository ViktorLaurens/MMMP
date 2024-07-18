"""

Degree PRM Planner for Pybullet Robot Classes

Notes: 
 - COUPLED approach
 - MULTI ROBOT USE

"""
from planners.prm.pybullet.coupled.coupled_prm import CoupledPRM
from planners.prm.planar.utils import Node
from scipy.spatial import KDTree

class KDTreeKNNCoupledPRM(CoupledPRM):
    """
    Implementation of Degree PRM Planner for Pybullet Robot Classes
    in which a KDTree is used for efficient nearest neighbor queries.
    (this requires all the nodes to be sampled first to build the KDTree!)
    """ 
    def __init__(self, environment, max_edge_len, n_knn):
        super().__init__(environment)
        self.max_edge_len = max_edge_len
        self.n_knn = n_knn

        n_samples = input("Enter the number of initial nodes: ")
        self.build_roadmap_n(int(n_samples))

        # print data
        nr_nodes = self.compute_combined_nr_nodes
        nr_edges = self.compute_combined_nr_edges
        avg_degree = self.compute_combined_avg_degree
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
            self.node_q_id_dict.update({new_node.q: new_node.id})
            self.node_id_q_dict.update({new_node.id: new_node.q})
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