"""

Utilities and tools for implementation of PRM-based planners

"""
from utils.planner_utils import Interval

# constants
INF = float("inf")

# PRM classes
# Comes from CBSPRM
class Node:
    def __init__(self, id, q):
        self.id = id
        self.q = tuple([round(c, 2) for c in q])
    def __eq__(self, other):
        return self.id == other.id
    def __lt__(self, other):
        return self.id < other.id
    def __hash__(self):
        return hash(str(self.id) + str(self.q))
    def __str__(self):
        return str(self.id, self.q)  
    
# Comes from CBSPRM
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

# Comes from CBSPRM
class State:
    def __init__(self, t, node):
        self.t = t
        # self.node = node
        # self.id = node.id
        self.q = node.q
    def __eq__(self, other):
        return self.t == other.t and self.q == other.q
    def __hash__(self):
        return hash(str(self.t) + str(self.q))
    def is_equal_except_time(self, state):
        return self.q == state.q
    def __str__(self):
        return str((self.t, self.q))
    
# PRM functions
# Comes from CBSPRM
def create_intervals_from_tuples(tuple1, tuple2):
    return [Interval(lower=min(a, b), upper=max(a, b)) for a, b in zip(tuple1, tuple2)]
 