"""

Probabilistic Road Map (PRM) Planner

author: Viktor Laurens De Groote

"""

from collections import deque
import datetime
import math
import time
from matplotlib.patches import Circle, Polygon, Rectangle
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

from utils.planner_utils import Interval

# Define the path to your project root directory
project_root = 'C:\\Users\\vikto\\Thesis'

# Add the project root directory to sys.path if it's not already included
import sys
import os
if project_root not in sys.path:
    sys.path.insert(0, project_root)



# parameter
N_SAMPLES = 100  # number of sample_points
MAX_TIME = 100
N_KNN = 5  # number of edge from one sampled point
MAX_EDGE_LEN = 4.0  # [m] Maximum edge length
TYPE = 'distance' # 'distance' or 'nn'
INF = float("inf")

class Node:
    def __init__(self, id, q):
        self.id = id
        self.q = tuple([round(c, 2) for c in q])
    def __eq__(self, other):
        return self.id == other.id
    def __hash__(self):
        return hash(str(self.id) + str(self.q))
    def __str__(self):
        return str(self.id, self.q)    
    
class PRM: 
    def __init__(self, config_space, obstacles=[], n_samples=N_SAMPLES, n_knn=N_KNN, max_edge_len=MAX_EDGE_LEN) -> None:
        self.config_space = config_space
        self.obstacles = obstacles
        self.n_samples = n_samples
        self.n_knn = n_knn
        self.max_edge_len = max_edge_len

        self.nodes = [] 
        self.agents = [] # Store agent's start and goal configs and nearest nodes to these configs


        self.node_id_config_dict = {}
        self.node_config_id_dict = {}

        self.nn_dict = {} #self.generate_neighbor_dict(self.nodes) # dict linking each node to neighbors using node indices

        # Build roadmap with N_SAMPLES
        self.build_roadmap(self.n_samples)
    
    # dictionaries for mapping id's and configurations
    def generate_node_id_config_dict(self, nodes):
        node_id_config_dict = {node.id: node.q for node in nodes}
        return node_id_config_dict
    
    def generate_node_config_id_dict(self, nodes):
        node_config_id_dict = {node.q: node.id for node in nodes}
        return node_config_id_dict
    
    # sampling functions
    def generate_samples(self, n): 
        # Sample n nodes
        sampled_nodes = np.random.uniform(
            [interval.lower for interval in self.config_space],
            [interval.upper for interval in self.config_space],
            (n, len(self.config_space))
        )
        
        # Round each element to 2 decimal places
        sampled_nodes = np.round(sampled_nodes, 2)

        # Return as Nodes 
        nodes = []
        for i, node in enumerate(sampled_nodes): 
            nodes.append(Node(i, node))
        return nodes
    
    def add_samples(self, n):
        # generate n nodes
        nodes = self.generate_samples(n)

        # add nodes to self.nodes
        for node in nodes: 
            self.nodes.append(node) 
        return 
    
    # building roadmap using local planner 
    def build_roadmap(self, n): 
        self.add_samples(n)
        self.update_roadmap() 
        return

    def update_roadmap(self): 
        self.nn_dict = self.update_nn_dict()
        self.node_id_config_dict = self.generate_node_id_config_dict(self.nodes)
        self.node_config_id_dict = self.generate_node_config_id_dict(self.nodes) 
        return

    def update_nn_dict(self, type=TYPE): 
        # Initialize nn_dict
        nn_dict = {node.id: [] for node in self.nodes}

        # Create a KDTree for efficient nearest neighbor search
        samples = [node.q for node in self.nodes]
        tree = KDTree(samples)
        # for distance PRM
        if type=='distance': 
            # Populate the knn_dict based on max_edge_len
            for i, node in enumerate(samples):
                # Query all points within max_edge_len radius
                indices = tree.query_ball_point(node, r=self.max_edge_len)
                for idx in indices:
                    if idx != i and self.is_collision_free(node, samples[idx], self.config_space): # local planner 
                        # Add to neighbor list
                        nn_dict[i].append(idx)
            return nn_dict 
        # for nn PRM
        elif type=='nn':
            # Populate the knn_dict based on max_edge_len
            for i, node in enumerate(samples):
                # Query all points within max_edge_len radius
                dists, indices = tree.query(node, k=self.n_knn+1)
                for idx in indices:
                    if idx != i and self.is_collision_free(node, samples[idx], self.config_space): # local planner
                        # Add to neighbor list
                        nn_dict[i].append(idx)
            return nn_dict 
        else: 
            raise Exception(f'type should be "distance" or "nn" but is {type} instead.')
    
    # local planner 
    def is_collision_free(self, start, end, obstacles):
        # Placeholder for collision check implementation
        # This should return False if the line from start to end intersects any obstacle
        return True

    def collision_free_node(self, node, obstacles):
        for obstacle in obstacles: 
            if obstacle.contains_point(node.q):
                return False
        return True
    
    def collision_free_edge(self, start, end, obstacles):
        # Placeholder for collision check implementation
        # This should return False if the line from start to end intersects any obstacle
        return True
    
    # agent handling
    def add_agent(self, agent):
        """Adds an agent to the PRM by finding the nearest roadmap nodes to its start and goal configurations."""
        start_config = agent["start"]
        goal_config = agent["goal"]
        
        # Assuming find_nearest_roadmap_node returns a Node object or None
        nearest_start_node = self.find_nearest_roadmap_node(start_config)
        nearest_goal_node = self.find_nearest_roadmap_node(goal_config)
        
        if nearest_start_node is None or nearest_goal_node is None:
            print(f"Could not find suitable start or goal nodes for agent {agent['name']}")
            return
        
        # Add additional agent information
        self.agents.append({
            "name": agent["name"],
            "start": start_config,
            "goal": goal_config,
            "start_node": nearest_start_node,
            "goal_node": nearest_goal_node
        })

    def add_agents(self, agents):
        """Adds each agent in a list of agents to the PRM."""
        for agent in agents: 
            self.add_agent(agent)
        return

    def find_nearest_roadmap_node(self, config):
        """Finds the nearest roadmap node to a given configuration by looping over all nodes."""
        min_dist = float('inf')
        nearest_node = None

        # Convert config to a numpy array for efficient distance computation
        config_array = np.array(config)

        for node in self.nodes:
            node_array = np.array(node.q)
            dist = np.linalg.norm(config_array - node_array)

            if dist < min_dist:
                min_dist = dist
                nearest_node = node

        return nearest_node
    
    # Querying roadmap
    def query(self):
        paths = {}
        for agent in self.agents:
            # find paths from nearest node to start and nearest node to goal 
            start_node = agent['start_node']
            goal_node = agent['goal_node']

            if start_node is None or goal_node is None:
                print(f"No path found for agent {agent['name']} due to missing start/goal nodes.")
                paths[agent['name']] = []
                continue

            # Use Dijkstra to find a path between the nearest start and goal nodes
            dijkstra_start_time = time.perf_counter()
            path, d_distance = self.dijkstra_search(start_node.id, goal_node.id)
            dijkstra_duration = time.perf_counter() - dijkstra_start_time
            print(f"Found path for agent {agent['name']} in {dijkstra_duration:.6f} seconds with distance {d_distance:.2f}")
            
            # Use A* to find a path between the nearest start and goal nodes
            a_star_start_time = time.perf_counter()
            path, a_distance = self.a_star_search(start_node.id, goal_node.id)
            a_star_duration = time.perf_counter() - a_star_start_time
            print(f"Found path for agent {agent['name']} in {a_star_duration:.6f} seconds with distance {a_distance:.2f}")
            
            # Store the path for the agent
            paths[agent['name']] = path

        return paths
    
    # Search methods
    def dijkstra_search(self, start_node_id, goal_node_id):
        """Perform Dijkstra's algorithm using node IDs."""
        # Initialize distances and previous node tracking
        distance_from_start = {node_id: INF for node_id in self.nn_dict.keys()}
        distance_from_start[start_node_id] = 0
        previous_node = {node_id: None for node_id in self.nn_dict.keys()}

        unvisited_nodes = set(self.nn_dict.keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Choose the unvisited node with the smallest distance
            current_node_id = min(unvisited_nodes, key=lambda node_id: distance_from_start[node_id])
            unvisited_nodes.remove(current_node_id)

            if distance_from_start[current_node_id] == INF:
                break  # Remaining nodes are unreachable

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            for neighbor_id in self.nn_dict[current_node_id]:
                neighbor_config = self.node_id_config_dict[neighbor_id]
                current_node_config = self.node_id_config_dict[current_node_id]
                # Calculate the distance between current node and neighbor
                distance = np.linalg.norm(np.array(neighbor_config) - np.array(current_node_config))

                new_distance = distance_from_start[current_node_id] + distance
                if new_distance < distance_from_start[neighbor_id]:
                    distance_from_start[neighbor_id] = new_distance
                    previous_node[neighbor_id] = current_node_id

        # Reconstruct the path from start_node_id to goal_node_id
        path = deque()
        current_node_id = goal_node_id
        while previous_node[current_node_id] is not None:
            path.appendleft(current_node_id)
            current_node_id = previous_node[current_node_id]
        path.appendleft(start_node_id)

        return list(path), distance_from_start[goal_node_id]

    def a_star_search(self, start_node_id, goal_node_id):
        """Perform A* search using node IDs."""
        # Initialize distances and previous node tracking
        distance_from_start = {node_id: INF for node_id in self.nn_dict.keys()}
        distance_from_start[start_node_id] = 0
        estimated_total_cost = {node_id: INF for node_id in self.nn_dict.keys()}
        estimated_total_cost[start_node_id] = self.heuristic(start_node_id, goal_node_id)
        previous_node = {node_id: None for node_id in self.nn_dict.keys()}

        unvisited_nodes = set(self.nn_dict.keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Choose the unvisited node with the smallest estimated total cost
            current_node_id = min(unvisited_nodes, key=lambda node_id: estimated_total_cost[node_id])
            unvisited_nodes.remove(current_node_id)

            if current_node_id == goal_node_id:
                break  # Destination reached

            # Check each neighbor of the current node
            for neighbor_id in self.nn_dict[current_node_id]:
                neighbor_config = self.node_id_config_dict[neighbor_id]
                current_node_config = self.node_id_config_dict[current_node_id]
                # Calculate the distance between current node and neighbor
                distance = np.linalg.norm(np.array(neighbor_config) - np.array(current_node_config))

                new_distance = distance_from_start[current_node_id] + distance
                if new_distance < distance_from_start[neighbor_id]:
                    distance_from_start[neighbor_id] = new_distance
                    estimated_total_cost[neighbor_id] = new_distance + self.heuristic(neighbor_id, goal_node_id)
                    previous_node[neighbor_id] = current_node_id

        # Reconstruct the path from start_node_id to goal_node_id
        path = deque()
        current_node_id = goal_node_id
        while previous_node[current_node_id] is not None:
            path.appendleft(current_node_id)
            current_node_id = previous_node[current_node_id]
        path.appendleft(start_node_id)

        return list(path), distance_from_start[goal_node_id]

    def heuristic(self, node_id1, node_id2):
        """Calculate the Euclidean distance between two nodes."""
        config1 = np.array(self.node_id_config_dict[node_id1])
        config2 = np.array(self.node_id_config_dict[node_id2])
        return np.linalg.norm(config1 - config2)

    # Visualizer
    def visualize_roadmap(self, solution=None):
        fig, ax = plt.subplots()  # Adjusted figsize
        
        # Plot nodes as blue dots
        for node in self.nodes:
            ax.plot(node.q[0], node.q[1], 'o', color='blue', markersize=3)

        # Highlight start and goal nodes
        for agent in self.agents: 
            start_q = agent['start']
            goal_q = agent['goal']
            ax.plot(start_q[0], start_q[1], 'o', color='red', markersize=4)
            ax.plot(goal_q[0], goal_q[1], 'o', color='green', markersize=4)
        
        # Draw edges between each node and its neighbors
        for node, neighbors in self.nn_dict.items():
            node_q = self.node_id_config_dict[node]
            for neighbor in neighbors:
                neighbor_q = self.node_id_config_dict[neighbor]
                ax.plot([node_q[0], neighbor_q[0]], [node_q[1], neighbor_q[1]], 'k-', linewidth=0.5)

        # Plot the solution paths if provided
        if solution is not None:
            for agent_id, path_node_ids in solution.items():
                path_locs = [self.node_id_config_dict[node_id] for node_id in path_node_ids]
                xs, ys = zip(*path_locs)
                ax.plot(xs, ys, marker='o', linestyle='-', linewidth=1.5, markersize=4, label=f"Path for {agent_id}")
            
        # Plot lines connecting start and goal nodes with their nearest roadmap nodes
        for agent in self.agents: 
            start_q = agent['start']
            goal_q = agent['goal']
            nearest_start_q = agent['start_node'].q
            nearest_goal_q = agent['goal_node'].q
            ax.plot([start_q[0], nearest_start_q[0]], [start_q[1], nearest_start_q[1]], 'r--', linewidth=1)
            ax.plot([goal_q[0], nearest_goal_q[0]], [goal_q[1], nearest_goal_q[1]], 'g--', linewidth=1)
        
        ax.set_aspect('equal', adjustable='box')
        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # plt.title('Roadmap Visualization')
        # plt.legend(loc='upper left')

        # Save figure as svg
        # now = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')  # Get current time to use in filename
        # if TYPE=='nn':
        #     plt.savefig(f'plots/roadmap_100samples_5knn_2agents_0obstacles_minimalistic_{now}_300dpi.eps', format='eps', dpi=300)
        # if TYPE=='distance':
        #     plt.savefig(f'plots/roadmap_100samples_4edgelength_2agents_0obstacles_minimalistic_{now}_300dpi.eps', format='eps', dpi=300)
        
        plt.show()


def prm_planning(cspace, agents, obstacles=None, n_samples=N_SAMPLES, max_time=MAX_TIME, visualizer=True): 
    # Initialize PRM
    prm = PRM(cspace, obstacles=obstacles, n_samples=n_samples)
    prm.add_agents(agents)
    paths = prm.query()
    if visualizer: 
        prm.visualize_roadmap(paths)
    return paths

def main():
    cspace = [Interval(lower=-10, upper=10), Interval(lower=-10, upper=10)]

    # agents
    start1 = [-9, -9]
    goal1 = [9, 9]
    start2 = [9, -9]
    goal2 = [-9, 9]
    agents = [
            {"name": "agent1", "start": start1, "goal": goal1, "model": None, "roadmap": None}, 
            {"name": "agent2", "start": start2, "goal": goal2, "model": None, "roadmap": None}
        ]
    
    # obstacles
    # obstacles = [Rectangle(center=[0, 0], width=4, height=4), Circle(center=[-5, 5], radius=2), 
    #              Polygon([[-5, -5], [-5, -3], [-3, -3], [-3, -5]])]
    
    paths = prm_planning(cspace, agents, obstacles=[])
    print(paths)

if __name__ == "__main__":
    main()