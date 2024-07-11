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
    
    def get_edge(self, node1: Node, node2: Node) -> Edge:
        return self.edges[(node1.id, node2.id)]
    
    def get_edges(self) -> List[Edge]:
        return list(self.edges.values())
    
    def get_nodes(self) -> List[Node]:
        return list(self.nodes.values())
    
    def get_neighbours(self, node: Node) -> List[Node]:
        neighbours = []
        for edge in self.get_edges():
            if edge.node1 == node:
                neighbours.append(edge.node2)
            elif edge.node2 == node:
                neighbours.append(edge.node1)
        return neighbours