import heapq
import numpy as np

class Node:
    def __init__(self, id, q):
        self.id = id
        self.q = q

    def __lt__(self, other):
        return self.id < other.id

class PRM:
    def __init__(self, k1, k2, r_ids):
        self.k1 = k1
        self.k2 = k2
        self.r_ids = r_ids
        self.node_lists = {r_id: [
            Node(0, [1,1]),
            Node(1, [1,2]),
            Node(2, [2,1]),
            Node(3, [10,10]),
            Node(4, [11,10]),
            Node(5, [10,11]),
            Node(6, [20,20]),
            Node(7, [21,20]),
            Node(8, [20,21]),
            Node(9, [30,30]),
            Node(10, [30,31]),
            Node(11, [31,30])
        ] for r_id in r_ids}
        self.edge_dicts = {r_id: {} for r_id in r_ids}
        for r_id in r_ids:
            for node in self.node_lists[r_id]: 
                self.edge_dicts[r_id][node.id] = []

    def distance(self, r_id, q1, q2):
        # Placeholder distance function (e.g., Euclidean distance)
        return np.linalg.norm(np.array(q1) - np.array(q2))

    def is_collision_free_edge(self, q1, q2, r_id):
        # Placeholder collision check (always returns True for simplicity)
        return True
    
    def heapsort(self, iterable):
        h = []
        for value in iterable:
            heapq.heappush(h, value)
        return [heapq.heappop(h) for _ in range(len(h))]

    def connect_nearest_neighbors_degree(self, r_id):
        # Connect nodes with k2 nearest neighbors, constructing the roadmap
        for node in self.node_lists[r_id]:
            # Determine k1 candidate neighbors
            candidate_neighbors = []
            for other_node in self.node_lists[r_id]:
                if node == other_node or len(self.edge_dicts[r_id][other_node.id]) == self.k2:
                    continue
                dist = self.distance(r_id, node.q, other_node.q)
                heapq.heappush(candidate_neighbors, (-dist, other_node))
                if len(candidate_neighbors) > self.k1:
                    heapq.heappop(candidate_neighbors)
            candidate_neighbors = [(-dist, node) for dist, node in candidate_neighbors]  # make dists > 0 again
            candidate_neighbors = self.heapsort(candidate_neighbors)
            # Connect the k2 nearest neighbors that can be connected with collision free edges
            for _, neighbor in candidate_neighbors:  # make sure the node itself is excluded
                if len(self.edge_dicts[r_id][node.id]) == self.k2:
                    break
                if self.is_collision_free_edge(node.q, neighbor.q, r_id) \
                        and neighbor.id not in self.edge_dicts[r_id][node.id] \
                        and len(self.edge_dicts[r_id][neighbor.id]) != self.k2:
                    self.edge_dicts[r_id][node.id].append(neighbor.id)
                    self.edge_dicts[r_id][neighbor.id].append(node.id)
        return
    
    def connect_nearest_neighbors_degree2(self, r_id):
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
    

# Example usage
prm = PRM(k1=5, k2=3, r_ids=['robot1'])
prm.connect_nearest_neighbors_degree('robot1')

# Print the resulting connections
for node_id, neighbors in prm.edge_dicts['robot1'].items():
    print(f"Node {node_id} connected to nodes: {neighbors}")
