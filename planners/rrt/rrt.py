import random
import time
from typing import Any, Dict, List, Optional
import numpy as np

from planners.rrt.utils import CollisionError

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
    def __init__(self, environment, build_type='n', n=100, t=10, maxdists=None, local_step_sizes=None, goal_sample_rate=0.1):
        self.r_ids = [environment.robot_models[i].r_id for i in range(len(environment.robot_models))]
        self.c_spaces = self.create_c_spaces(environment.robot_models)

        self.build_type = build_type
        self.n = n
        self.t = t

        # unpack environmental data
        self.env = environment
        self.agents = environment.agents                
        self.robot_models = environment.robot_models    
        self.obstacles = environment.obstacles

        self.maxdists = maxdists
        if maxdists is None: 
            self.maxdists = self.calc_maxdists(self.r_ids, self.c_spaces.values(), res=100)

        self.local_step_sizes = local_step_sizes
        if local_step_sizes is None:
            self.local_step_sizes = {r_id: maxdist / 10 for r_id, maxdist in self.maxdists.items()}

        self.goal_sample_rate = goal_sample_rate

        # Create trees for each robot
        self.trees = {r_id: Tree(Node(0, environment.robot_models[r_id].get_arm_pose())) for r_id in self.r_ids}


    # def create_c_spaces(self, robot_models):
    #     c_spaces = []
    #     for model in robot_models:
    #         c_spaces.append(model.arm_c_space)  # specific c-space of franka panda robot arm
    #     return c_spaces
    
    def create_c_spaces(self, robot_models):
        c_spaces = {}
        for model in robot_models:
            c_spaces.update({model.r_id: model.arm_c_space})  # specific c-space of franka panda robot arm
        return c_spaces
    
    def calc_maxdists(self, r_ids, c_spaces, res):
        assert len(r_ids) == len(c_spaces)
        maxdists = {}
        for r_id, c_space in zip(r_ids, c_spaces): 
            unit_vector = np.array([])
            for interval in c_space: 
                unit_vector = np.append(unit_vector, (interval.upper - interval.lower) / res + interval.lower)
            maxdists.update({r_id: self.distance(r_id, unit_vector, [interval.lower for interval in c_space])})
        return maxdists

    def reset_robot_poses(self):
        for i, robot in enumerate(self.robot_models):
            robot.set_arm_pose(self.agents[i]["start"])
        return
    
    # Sampling
    def sample_random_q(self, c_space):
        q = np.random.uniform(low=[i.lower for i in c_space], high=[i.upper for i in c_space])
        return q

    # def is_collision_free_composite_q(self, qs):
    #     for i, q in enumerate(qs):
    #         self.robot_models[i].set_arm_pose(q)
    #     for robot in self.robot_models:
    #         if self.env.robot_collision(robot):
    #             return False
    #     return True
    
    def sample_valid_random_q(self, r_id, c_space):
        q = self.sample_random_q(c_space)
        self.robot_models[r_id].set_arm_pose(q)
        while self.env.robot_collision(self.robot_models[r_id]):
            q = self.sample_random_q(c_space)
            self.robot_models[r_id].set_arm_pose(q)
        return q
    
    def sample_valid_random_qs(self, robots, c_spaces):
        return [self.sample_valid_random_q(robot.r_id, c_space) for robot, c_space in zip(robots, c_spaces)]
    
    # Distance
    def distance(self, r_id, q1, q2):
        return self.specific_distance(r_id, q1, q2)
    
    def general_distance(self, q1, q2):
        return np.linalg.norm(np.array(q1) - np.array(q2))

    def specific_distance(self, r_id, q1, q2):
        index = self.r_ids.index(r_id)
        model = self.robot_models[index]
        return model.panda_specific_distance_metric(q1, q2)
    
    def composite_distance(self, qs_1, qs_2):
        d = 0
        for q1, q2 in zip(qs_1, qs_2):
            d += self.distance(q1, q2) 
        return d
    
    # Nearest
    def nearest_node_id(self, r_id, q):
        tree = self.trees[r_id]
        nearest_node_id = min(tree.nodes.keys(), key=lambda x: self.distance(r_id, q, tree.nodes[x].q))
        # nearest_q = np.array(tree.nodes[nearest_node_id].q)
        # while not collision_free_segment(q, tree.nodes[nearest_node_id].q):
        #     nearest_node_id = min(tree.nodes.keys(), key=lambda x: self.distance(q, tree.nodes[x].q))
        return nearest_node_id

    # def nearest_node_id(self, q, tree):
    #     nearest_node_id = min(tree.nodes.keys(), key=lambda x: self.distance(q, tree.nodes[x].q))
    #     # If the nearest node is not collision-free, find the next nearest one.
    #     if not self.collision_free_segment(q, tree.nodes[nearest_node_id].q):
    #         sorted_node_ids = sorted(tree.nodes.keys(), key=lambda x: self.distance(q, tree.nodes[x].q))
    #         for node_id in sorted_node_ids[1:]:
    #             if self.collision_free_segment(q, tree.nodes[node_id].q):
    #                 nearest_node_id = node_id
    #                 break
    #         else:
    #             # Raise an exception if no collision-free path is found (optional)
    #             raise ValueError("No collision-free path found to any node.")
    #     return nearest_node_id

    def composite_nearest_node_ids(self, qs):
        composite_nearest = []
        for i, r_id in enumerate(self.r_ids):
            composite_nearest.append(self.nearest_node_id(r_id, qs[i]))
        return composite_nearest
     
    # Extending
    def extend(self, r_id, q_nearest, q, maxdist):
        direction = np.array(q) - np.array(q_nearest)
        norm = self.distance(r_id, q_nearest, q)
        q_new = np.array(q_nearest) + maxdist * direction / norm 
        return q_new
    
    def composite_extend(self, q_nearests, qs, maxdists):
        composite_extend = []
        for i, r_id in enumerate(self.r_ids):
            composite_extend.append(self.extend(r_id, q_nearests[i], qs[i], maxdists[r_id]))
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
    
    def composite_collision_free_start(self):
        errors = []
        for i, robot in enumerate(self.robot_models):
            robot.set_arm_pose(self.agents[i]["start"])
        for i, robot in enumerate(self.robot_models):
            if self.env.robot_collision(robot):
                errors.append(f"Agent {self.agents[i]['name']} start configuration is in collision.")
        if errors:
            raise CollisionError('\n'.join(errors))
        return True
    
    def composite_collision_free_goal(self):
        errors = []
        for i, robot in enumerate(self.robot_models):
            robot.set_arm_pose(self.agents[i]["goal"])
        for i, robot in enumerate(self.robot_models):
            if self.env.robot_collision(robot):
                errors.append(f"Agent {self.agents[i]['name']} goal configuration is in collision.")
        if errors:
            raise CollisionError('\n'.join(errors))
        return True
    
    def composite_collision_free_segment(self, q_nearests, q_news):
        res = int(self.maxdists[0] / self.local_step_sizes[0]) # local_step_size is derived as a fraction of maxdist, here we calculate this fraction from local_step_sizes and maxdists
        segments = {}
        for i, r_id in enumerate(self.r_ids):
            segments.update({r_id: np.linspace(q_nearests[i], q_news[i], res)})
        for j in range(res):
            qs = [segment[j] for segment in segments.values()]
            if not self.composite_collision_free(qs):
                return False
        return True

    # Query
    def goal_reached(self, qs):
        for i, r_id in enumerate(self.r_ids):
            if self.distance(r_id, qs[i], self.agents[i]["goal"]) > self.maxdists[i]:
                return False
        return True

    def query(self): 
        if self.build_type == 'n':
            trees = self.query_n()
            n_id_paths = self.paths_from_trees()
            paths = self.paths_in_time(n_id_paths)
            return paths
        elif self.build_type == 't':
            trees = self.query_t()
            n_id_paths = self.paths_from_trees()
            paths = self.paths_in_time(n_id_paths)
            return paths
        else:
            raise ValueError("Invalid build type. Enter 'n' for number of iterations or 't' for time limit.")
    
    def query_n(self):
        # Check for collisions at the start and goal configurations
        try:
            self.composite_collision_free_start()
            self.composite_collision_free_goal()
            self.reset_robot_poses()
        except CollisionError as e:
            print(e)  # Print the exception message
            return None  # Exit the planning function if a collision is detected
        
        for i in range(self.n):
            qs = [self.agents[i]["goal"] for i in range(len(self.robot_models))] if (np.random.rand() < self.goal_sample_rate) else self.sample_valid_random_qs(self.robot_models, self.c_spaces.values())
            nearests_node_ids = self.composite_nearest_node_ids(qs)
            q_nearests = [np.array(tree.nodes[nearest_node_id].q) for tree, nearest_node_id in zip(self.trees.values(), nearests_node_ids)] 
            q_news = self.composite_extend(q_nearests, qs, self.maxdists)
            if self.composite_collision_free_segment(q_nearests, q_news):
                for i, q_new in enumerate(q_news):
                    self.trees[i].add_node(Node(len(self.trees[i].nodes), q_new))
                    self.trees[i].add_edge(self.trees[i].get_node(nearests_node_ids[i]), self.trees[i].get_node(len(self.trees[i].nodes)-1))
                    if self.goal_reached(q_news):
                        self.trees
                        return self.trees
        raise ValueError(f"RRT could not find a path with {self.n} iterations.")

    def query_t(self):
        # Check for collisions at the start and goal configurations
        try:
            self.composite_collision_free_start()
            self.composite_collision_free_goal()
            self.reset_robot_poses()
        except CollisionError as e:
            print(e)  # Print the exception message
            return None
        
        start_time = time.time()
        while time.time() - start_time < self.t:
            qs = [self.agents[i]["goal"] for i in range(len(self.robot_models))] if (np.random.rand() < self.goal_sample_rate) else self.sample_valid_random_qs(self.robot_models, self.c_spaces.values())
            nearests_node_ids = self.composite_nearest_node_ids(qs)
            q_nearests = [np.array(tree.nodes[nearest_node_id].q) for tree, nearest_node_id in zip(self.trees.values(), nearests_node_ids)] 
            q_news = self.composite_extend(q_nearests, qs, self.maxdists)
            # q_news possibly not in config space, problem? 
            if self.composite_collision_free_segment(q_nearests, q_news):
                for i, r_id in enumerate(self.r_ids):
                    q_new = q_news[i]
                    self.trees[r_id].add_node(Node(len(self.trees[r_id].nodes), q_new))
                    self.trees[r_id].add_edge(self.trees[r_id].get_node(nearests_node_ids[i]), self.trees[r_id].get_node(len(self.trees[r_id].nodes)-1))
                    if self.goal_reached(q_news):
                        # Add goal nodes to trees 
                        for j, r_id in enumerate(self.r_ids):
                            self.trees[r_id].add_node(Node(-1, self.agents[j]["goal"]))
                            self.trees[r_id].add_edge(self.trees[r_id].get_node(len(self.trees[r_id].nodes)-2), self.trees[r_id].get_node(-1))
                        return self.trees
        raise ValueError(f"RRT could not find a path in {self.t} seconds.")
    
    def paths_from_trees(self):
        paths = []
        for i in range(len(self.robot_models)):
            paths.append(self.trees[i].trace_path(-1))
        return paths
    
    def paths_in_time(self, n_id_paths, time_step=0.01):
        paths = {r_id: {} for r_id in self.r_ids}
        for i, r_id in enumerate(self.r_ids):
            n_id_path = n_id_paths[i]
            maxdist = self.maxdists[r_id]
            local_step_size = self.local_step_sizes[r_id]
            for j, n_id in enumerate(n_id_path):
                q = self.trees[r_id].nodes[n_id].q
                paths[r_id][round(maxdist / local_step_size * time_step * j, 2)] = q   # goal node in n_id_path!!!!
        self.discretize_paths(paths, time_step)
        return paths

    def discretize_paths(self, paths, time_step):
        for r_id, path in paths.items():
            max_t = max(path.keys())
            for t in np.round(np.arange(time_step, max_t, time_step), 2):
                if t not in path:
                    t_prev = max([k for k in path.keys() if k < t])
                    t_next = min([k for k in path.keys() if k > t])
                    q_prev = path[t_prev]
                    q_next = path[t_next]
                    q = self.interpolate(q_prev, q_next, t_prev, t_next, t)
                    path[t] = q
        return paths
    
    def interpolate(self, q1, q2, t1, t2, t):
        return np.array(q1) + (np.array(q2) - np.array(q1)) * (t - t1) / (t2 - t1)
    

    







