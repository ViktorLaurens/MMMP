import random
from typing import Any, Dict, List, Optional

import numpy as np

class Node:
    def __init__(self, id, q) -> None:
        self.id = id
        self.q = tuple([round(c, 2) for c in q])
    def __eq__(self, other):
        return self.id == other.id
    def __hash__(self):
        return hash(str(self.id) + str(self.q))
    def __str__(self):
        return str(self.id, self.q) 

class Edge:
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2
    def __eq__(self, other):
        return self.node1 == other.node1 and self.node2 == other.node2
    def __hash__(self):
        return hash(str(self.node1) + str(self.node2))
    def __str__(self):
        return str((self.node1, self.node2))
    
class State:
    def __init__(self, t, node):
        self.t = t
        self.q = node.q
    def __eq__(self, other):
        return self.t == other.t and self.q == other.q
    def __hash__(self):
        return hash(str(self.t) + str(self.q))
    def is_equal_except_time(self, state):
        return self.q == state.q
    def __str__(self):
        return str((self.t, self.q))

class Tree: 
    def __init__(self, root: Node) -> None:
        self.root = root
        self.nodes = {root.id: root}
        self.edges = {}
    
    def add_node(self, node: Node) -> None:
        self.nodes[node.id] = node
    
    def add_edge(self, parent: Node, child: Node) -> None:
        self.edges.update({child.id: parent.id})
    
    def get_node(self, id: int) -> Node:
        return self.nodes[id]
    
    def get_parent_id(self, child_id: int) -> int:
        return self.edges[child_id]
    
    def trace_path(self, leaf_id: int) -> List[int]:
        path = [leaf_id]
        child_id = leaf_id
        while child_id in self.edges:
            parent_id = self.get_parent_id(child_id)
            path.append(parent_id)    
            child_id = parent_id
        return path[::-1]
    
    # def get_edge(self, node1: Node, node2: Node) -> Edge:
    #     return self.edges[(node1.id, node2.id)]
    
    # def get_edges(self) -> List[Edge]:
    #     return list(self.edges.values())
    
    # def get_nodes(self) -> List[Node]:
    #     return list(self.nodes.values())
    
    # def get_neighbours(self, node: Node) -> List[Node]:
    #     neighbours = []
    #     for edge in self.get_edges():
    #         if edge.node1 == node:
    #             neighbours.append(edge.node2)
    #         elif edge.node2 == node:
    #             neighbours.append(edge.node1)
    #     return neighbours

class RRT:
    """
    Class for RRT motion planning in pybullet. 
    """
    def __init__(self, environment, maxdist, build_type='n', n=100, t=10, local_step=0.05, goal_sample_rate=0.1):
        self.robot_ids = np.linspace(0, len(environment.robot_models)-1, len(environment.robot_models), dtype=int)
        self.c_spaces = self.create_c_spaces(environment.robot_models)
        self.maxdist = maxdist
        self.build_type = build_type
        self.n = n
        self.t = t
        self.local_step = local_step
        self.goal_sample_rate = goal_sample_rate

        # Create trees for each robot
        self.trees = {r_id: Tree(Node(0, environment.robot_models[r_id].get_arm_pose())) for r_id in self.robot_ids}

        # unpack environmental data
        self.env = environment
        self.agents = environment.agents                
        self.robot_models = environment.robot_models    
        self.obstacles = environment.obstacles

    def create_c_spaces(self, robot_models):
        c_spaces = []
        for model in robot_models:
            c_spaces.append(model.arm_c_space)  # specific c-space of franka panda robot arm
        return c_spaces
    
    # Sampling
    def sample_random_q(self, c_space):
        q = np.random.uniform(low=[i.lower for i in c_space], high=[i.upper for i in c_space])
        return q
    
    def sample_valid_random_q(self, c_space):
        q = self.sample_random_q(c_space)
        while self.env.robot_collision(q):
            q = self.sample_random_q(c_space)
        return q
    
    def sample_valid_random_qs(self, c_spaces):
        return [self.sample_valid_q(c_space) for c_space in c_spaces]
    
    # Distance
    def distance(self, q1, q2):
        return self.general_distance(q1, q2)
    
    def general_distance(self, q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))

    def specific_distance(self, q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))
    
    def composite_distance(self, qs_1, qs_2):
        d = 0
        for q1, q2 in zip(qs_1, qs_2):
            d += self.distance(q1, q2) 
        return d
    
    # Nearest
    def nearest(self, q, tree):
        return min(tree.nodes.keys(), key=lambda x: self.distance(q, tree.nodes[x].q))

    def composite_nearest(self, qs):
        composite_nearest = []
        for i, tree in enumerate(self.trees):
            composite_nearest.append(self.nearest(qs[i], tree))
        return composite_nearest
     
    # Extending
    def extend(self, q_nearest, q):
        direction = np.array(q) - np.array(q_nearest)
        norm = np.linalg.norm(direction)
        q_new = np.array(q_nearest) + self.maxdist * direction / norm 
        return q_new
    
    def composite_extend(self, q_nearests, qs):
        composite_extend = []
        for i, q_nearest in enumerate(q_nearests):
            composite_extend.append(self.extend(q_nearest, qs[i]))
        return composite_extend
    
    # Collision checking
    def composite_collision_free(self, qs):
        for i, q in enumerate(qs):
            robot = self.robot_models[i]
            robot.set_arm_pose(q)
        for robot in self.robot_models:
            if self.env.robot_collision(robot):
                return False
        return True
    
    # Query
    def query(self): 
        if self.build_type == 'n':
            return self.query_n()
        elif self.build_type == 't':
            return self.query_t()
        else:
            raise ValueError("Invalid build type. Enter 'n' for number of iterations or 't' for time limit.")
    
    def query_n(self):
        pass

    def query_t(self):
        pass
    






