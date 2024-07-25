import heapq
import math
import random

class Node:
    def __init__(self, node_id, q):
        self.id = node_id
        self.q = q

class Roadmap:
    def __init__(self):
        self.r_ids = ['roadmap_1']
        self.node_lists = [[]]
        self.edge_dicts = [{}]

    def distance(self, r_id, q1, q2):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(q1, q2)))

    def is_collision_free_edge(self, q1, q2, r_id):
        # Mock implementation: assume all edges are collision-free
        return True

    def connect_nearest_neighbors_distance(self, r_id):
        r_index = self.r_ids.index(r_id)
        # Mock implementation to set up initial components
        nodes = self.node_lists[r_index]
        for node in nodes:
            for other_node in nodes:
                if node != other_node:
                    self.edge_dicts[r_index].setdefault(node.id, [])
                    self.edge_dicts[r_index].setdefault(other_node.id, [])
                    dist = self.distance(r_id, node.q, other_node.q)
                    if dist < 5:  # arbitrary threshold
                        self.edge_dicts[r_index][node.id].append(other_node.id)
                        self.edge_dicts[r_index][other_node.id].append(node.id)
    
    def compute_connected_components(self, r_id):
        r_index = self.r_ids.index(r_id)
        visited = set()
        components = []

        def dfs(node, component):
            stack = [node]
            while stack:
                current = stack.pop()
                if current.id not in visited:
                    visited.add(current.id)
                    component.append(current)
                    for neighbor_id in self.edge_dicts[r_index][current.id]:
                        neighbor = next(n for n in self.node_lists[r_index] if n.id == neighbor_id)
                        if neighbor.id not in visited:
                            stack.append(neighbor)

        for node in self.node_lists[r_index]:
            if node.id not in visited:
                component = []
                dfs(node, component)
                components.append(component)

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

# Example usage
roadmap = Roadmap()

# Adding nodes to the roadmap
roadmap.node_lists[0].extend([
    Node(1, (0, 0)),
    Node(2, (1, 1)),
    Node(3, (2, 2)),
    Node(4, (10, 10)),
    Node(5, (11, 11)),
    Node(6, (12, 12)), 
    Node(7, (20, 20)),
    Node(8, (21, 21)),
    Node(9, (22, 22))
    # Node(10, (5, 5)),
    # Node(11, (3, 4)),
    # Node(12, (6, 7)),
    # Node(13, (8, 9))
])

# Initial edges (for simplicity, connect close nodes)
roadmap.connect_nearest_neighbors_distance('roadmap_1')

# Compute connected components
components = roadmap.compute_connected_components('roadmap_1')
print("Components before connection:")
for comp in components:
    print([node.id for node in comp])

# Connect connected components
components = roadmap.connect_connected_components_random(components, 'roadmap_1')
print("\nComponents after connection:")
for comp in components:
    print([node.id for node in comp])
