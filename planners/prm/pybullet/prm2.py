import time

from matplotlib import pyplot as plt
import numpy as np
import pybullet as p
from scipy.spatial import KDTree

from planners.prm.pybullet.utils import Node

class ProbabilisticRoadMap:
    def __init__(self, environment, r_id, c_space, maxdist, k1=20, k2=10, build_type='kdtree', n=100, t=10, time_step=0.1, local_step=0.05) -> None:
        self.r_id = r_id
        self.c_space = c_space
        self.maxdist = maxdist
        self.k1 = k1
        self.k2 = k2
        self.build_type = build_type
        self.n = n
        self.t = t
        self.nodes = []
        self.edges = {}
        self.node_id_q_dict = {}
        self.node_q_id_dict = {}

        # collision checking
        self.time_step = time_step
        self.local_step = local_step

        # unpack environmental data
        self.env = environment
        self.agents = environment.agents                # This PRM class expects only 1 agent/robot in environment 
        self.robot_models = environment.robot_models    # This PRM class expects only 1 agent/robot in environment
        self.obstacles = environment.obstacles

        # call method to generate roadmap
        self.generate_roadmap()
    
    def generate_roadmap(self):
        # Clear roadmap
        self.nodes = []
        self.edges = {}

        # Build roadmap based on time
        if self.build_type == 't':
            start_time = time.time()
            while time.time() - start_time < self.t:
                sample = np.random.uniform([interval.lower for interval in self.c_space], 
                                            [interval.upper for interval in self.c_space], 
                                            len(self.c_space)
                                            )
                sample = np.round(sample, 2)
                new_node = Node(len(self.nodes), sample)
                self.nodes.append(new_node)
                self.edges.update({new_node.id: []})
                self._find_and_connect_neighbors(new_node)
            self.node_id_q_dict = self.generate_node_id_q_dict(self.nodes)
            self.node_q_id_dict = self.generate_node_q_id_dict(self.nodes)
            return
        
        # build roadmap based on number of nodes
        elif self.build_type == 'n':
            for i in range(self.n):
                sample = np.random.uniform([interval.lower for interval in self.c_space], 
                                            [interval.upper for interval in self.c_space], 
                                            len(self.c_space)
                                            )
                sample = np.round(sample, 2)
                new_node = Node(i, sample)
                self.nodes.append(new_node)
                self.edges.update({new_node.id: []})
                self._find_and_connect_neighbors(new_node)
            self.node_id_q_dict = self.generate_node_id_q_dict(self.nodes)
            self.node_q_id_dict = self.generate_node_q_id_dict(self.nodes)
            return
        
        # build roadmap based on kdtree, nodes are set a priori, no update of tree possible once created
        elif self.build_type == 'kdtree':

            # sample nodes
            samples = np.random.uniform([interval.lower for interval in self.c_space],
                                        [interval.upper for interval in self.c_space],
                                        (self.n, len(self.c_space))
                                        )
            samples = np.round(samples, 2)
            self.nodes = [Node(i, sample) for i, sample in enumerate(samples)]
            self.node_id_q_dict = self.generate_node_id_q_dict(self.nodes)
            self.node_q_id_dict = self.generate_node_q_id_dict(self.nodes)

            # create roadmap
            self.edges = {node.id: [] for node in self.nodes}
            tree = KDTree(samples)  
            for i, node in enumerate(self.nodes):
                id = node.id
                q = node.config
                # neighbors = tree.query_ball_point(q, r=self.maxdist)
                distances, neighbor_idxs = tree.query(q, k=self.k1+1) # +1 to exclude the node itself
                for d, id2 in zip(distances, neighbor_idxs):
                    if d > 0 \
                    and d < self.maxdist \
                    and len(self.edges[id]) < self.k1 \
                    and self.collision_free_edge(q, self.node_id_q_dict[id2]):
                        self.edges[id].append(id2)

        # raise error if build_type is not 't' or 'n' or 'kdtree'
        else:
            raise ValueError(f"Unexpected build_type {self.build_type}. Expected 't' or 'n' or 'kdtree'.")

    def query(self, start_q, goal_q):
        # Placeholder for search algorithm implementation
        pass

    def _find_and_connect_neighbors(self, new_node):
        candidate_neighbors = []
        distances = [(self.distance(new_node.config, node.config), node) 
                     for node in self.nodes[:-1]]
        distances.sort()  # Sort by distance
        
        for distance, neighbor in distances:
            if distance > self.maxdist or len(candidate_neighbors) == self.k1:
                break
            if not self._same_component(new_node, neighbor):
                candidate_neighbors.append(neighbor)
                
        for neighbor in candidate_neighbors:
            self._try_connect(new_node, neighbor)

    def _same_component(self, node1, node2):
        return False
    
    def _try_connect(self, node1, node2):
        if self.is_collision_free(node1, node2):
            self.edges[node1.id].append(node2.id)
            self.edges[node2.id].append(node1.id)
    
    def collision_free_edge(self, q1, q2):
        # Placeholder for collision check implementation with static obstacles in environment
        return True
    
    def generate_node_id_q_dict(self, nodes):
        return {node.id: node.config for node in nodes}
    
    def generate_node_q_id_dict(self, nodes):
        return {tuple(node.config): node.id for node in nodes}
    
    def distance(self, q1, q2):
        # Placeholder for distance calculation between two configurations
        return np.linalg.norm(np.array(q1) - np.array(q2))
    
    def add_start_and_goal(self, start_config, goal_config):
        raise NotImplementedError()
        # Ensure start and goal are numpy arrays for distance calculations
        start_node = np.array(start_config)
        self.start_nodes.append(start_node)
        goal_node = np.array(goal_config)
        self.goal_nodes.append(goal_node)

        # Initialize minimum distances to a large value
        min_dist_start, min_dist_goal = np.inf, np.inf
        nearest_start, nearest_goal = None, None
        
        # Iterate over each node to find the nearest node to start and goal
        for node in self.nodes:
            dist_to_start = np.linalg.norm(node - start_node)
            dist_to_goal = np.linalg.norm(node - goal_node)

            # Update nearest start node
            if dist_to_start < min_dist_start and self.is_collision_free(start_node, node, self.config_space):
                min_dist_start = dist_to_start
                nearest_start = node

            # Update nearest goal node
            if dist_to_goal < min_dist_goal and self.is_collision_free(goal_node, node, self.config_space):
                min_dist_goal = dist_to_goal
                nearest_goal = node

        # Add start and goal nodes to the roadmap if they have a nearest node within max_edge_len
        if nearest_start is not None:
            self.knn_dict[tuple(start_node)] = [nearest_start.tolist()]
            self.knn_dict[tuple(nearest_start)].append(start_node.tolist())

        if nearest_goal is not None:
            self.knn_dict[tuple(goal_node)] = [nearest_goal.tolist()]
            self.knn_dict[tuple(nearest_goal)].append(goal_node.tolist())

    def visualize_roadmap(self, solution=None):
        raise NotImplementedError()
        fig, ax = plt.subplots()
        
        # Plot nodes as blue dots
        for node in self.nodes:
            ax.plot(node[0], node[1], 'o', color='blue')
        
        # Draw edges between each node and its neighbors
        for node, neighbors in self.knn_dict.items():
            for neighbor in neighbors:
                ax.plot([node[0], neighbor[0]], [node[1], neighbor[1]], 'k-', linewidth=0.5)
        
        # Highlight start and goal nodes
        for start_node in self.start_nodes:
            ax.plot(start_node[0], start_node[1], 'o', color='green', markersize=10)  # Larger green dots for start nodes
        for goal_node in self.goal_nodes:
            ax.plot(goal_node[0], goal_node[1], 'o', color='red', markersize=10)  # Larger red dots for goal nodes

        # Plot the solution paths if provided
        if solution is not None:
            for agent_id, path in solution.items():
                path_locs = [step['loc'] for step in path]
                xs, ys = zip(*path_locs)
                ax.plot(xs, ys, marker='o', linestyle='-', label=f"Path for {agent_id}")
        
        ax.set_aspect('equal', adjustable='box')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Roadmap Visualization')
        plt.legend()
        plt.show()

    #  Astar search
    def a_star_search(self, constraints):
        # constants
        start_id = -1
        goal_id = -2

        # compute set of conflict times for more efficient collision checking
        conflict_times = set()
        constraints_from_t = {constraint.t: [] for constraint in constraints}
        for constraint in constraints:
            conflict_times.add(constraint.t)
            constraints_from_t[constraint.t].append(constraint)
    
        came_from = {}
        g_score = {node.id: np.inf for node in self.nodes}
        g_score[self.start_id] = 0
        f_score = {node.id: np.inf for node in self.nodes}
        f_score[self.start_id] = self.heuristic(self.start_id, self.goal_id)

        unvisited_nodes = set(self.edges.keys())  # Track unvisited nodes

        while unvisited_nodes:
            # Get the node with the lowest f score
            current_id = min(unvisited_nodes, key=lambda node_id: f_score[node_id])
            unvisited_nodes.remove(current_id)

            if g_score[current_id] == np.inf: # Remaining nodes are unreachable
                return [], np.inf # No path found

            if current_id == goal_id: # Check if the goal has been reached
                return self.reconstruct_path(current_id, came_from) # Path found, reconstruct the path from start to goal

            # Check each neighbor of the current node
            for neighbor_id in self.edges[current_id]:
                if not self.transition_valid(current_id, neighbor_id, conflict_times, constraints_from_t):
                    continue

                # Calculate the distance between current node and neighbor
                distance = self.distance(current_id, neighbor_id)

                # Update the distance from start to neighbor
                new_distance = g_score[current_id] + distance
                if new_distance < g_score[neighbor_id]:
                    g_score[neighbor_id] = new_distance
                    f_score[neighbor_id] = new_distance + self.heuristic(neighbor_id, goal_id)
                    came_from[neighbor_id] = current_id
        if g_score[goal_id] == np.inf:
            return [], np.inf # No path found
        else: 
            return self.reconstruct_path(goal_id, came_from), g_score[goal_id] # Path found, reconstruct the path from start to goal

    def heuristic(self, node_id1, node_id2):
        q1 = np.array(self.node_id_q_dict[node_id1])
        q2 = np.array(self.node_id_q_dict[node_id2])
        return self.distance(q1, q2)
    
    def reconstruct_path(self, current_id, came_from):
        path = [current_id]
        while current_id in came_from.keys():
            current_id = came_from[current_id]
            path.append(current_id)
        return path[::-1]
    
    def transition_valid(self, d_to_n_1, n_1_id, n_2_id, conflict_times, constraints_from_t):
        # Get the configurations for the current node and the neighbor node
        q1 = np.array(self.node_id_q_dict[n_1_id])
        q2 = np.array(self.node_id_q_dict[n_2_id])

        # Calculate the time interval between the current node and the neighbor node
        t1 = float(d_to_n_1 / self.local_step) * self.time_step
        t2 = t1 + np.linalg.norm(q2 - q1) / self.local_step * self.time_step
        delta_t = t2 - t1

        # Append the start time to the current and neighbor configurations
        q1_t1 = np.append(q1, t1)
        q2_t2 = np.append(q2, t2)

        # Calculate the start and end times for the interval
        interval_t1 = np.round(np.ceil(t1 / self.time_step) * self.time_step, 1)
        interval_t2 = np.round(np.floor(t2 / self.time_step) * self.time_step, 1)

        # Discretize the path from the start configuration to the end configuration
        discretized_path = {}
        for t in np.round(np.arange(interval_t1, interval_t2 + self.time_step, self.time_step), 1):   # np_arange(start, stop) -> [start, stop) 
            q_at_t = self.find_q_in_q_t_interval_given_t(q1_t1, q2_t2, t)
            discretized_path.update({np.round(t, 1): q_at_t})
            if t in conflict_times: 
                constraints = constraints_from_t[t]
                for constraint in constraints:
                    r_2_q = constraint.q2
                    if self.robot_robot_collision(q_at_t, r_2_q, constraint.r_2_id):
                        return False       
        return True
    
    def find_q_in_q_t_interval_given_t(self, q1_t1, q2_t2, t):
        delta_t = q2_t2[-1] - q1_t1[-1]
        x = (t - q1_t1[-1]) / delta_t
        return np.round(q1_t1[:-1] + (q2_t2[:-1] - q1_t1[:-1]) * x, 2)
    
    def robot_robot_collision(self, r_1_q, r_2_q, r_2_id):
        # Set the pose of the robot model in pybullet to location for collision checking
        self.robot_models[self.r_id].set_arm_pose(self.robot_models[self.r_id].joints, tuple(r_1_q))
        self.robot_models[r_2_id].set_arm_pose(self.robot_models[r_2_id].joints, tuple(r_2_q))
        # Check for collision between the two robots
        closest_points = p.getClosestPoints(bodyA=self.robot_models[self.r_id].robot_id, bodyB=self.robot_models[r_2_id].robot_id, distance=0.0)
        if closest_points:
            return True
        return False