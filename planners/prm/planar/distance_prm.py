"""

Distance PRM Planner for Planar Robots

Notes: 

 - SINGLE ROBOT USE: This PRM class expects only 1 agent/robot in the environment.

"""

import time
from planners.prm.planar.env import LOCAL_STEP
from planners.prm.planar.prm import PRM
from planners.prm.planar.utils import Node


class DistancePRM(PRM):
    def __init__(self, environment, max_edge_len, build_type='n'):
        super().__init__(environment, local_step=LOCAL_STEP)
        self.max_edge_len = max_edge_len
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
    
    def add_n_samples(self, n):
        for _ in range(n):
            sample = self.generate_free_sample()
            new_node = Node(id=len(self.nodes), q=sample)
            self.nodes.append(new_node)
            self.roadmap.update({new_node.id: []})
            self.node_config_id_dict.update({new_node.q: new_node.id})
            self.node_id_config_dict.update({new_node.id: new_node.q})
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
            self.node_config_id_dict.update({new_node.q: new_node.id})
            self.node_id_config_dict.update({new_node.id: new_node.q})
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
        return self.specific_distance(q1, q2)