"""

Coupled PRM Planner for Planar Robots

Notes: 

 - BLUEPRINT: This PRM class cannot be used as a standalone planner BUT
is a blueprint for more specific PRM classes that inherit from it.

 - MULTI ROBOT USE: This PRM class can handle 1 or more robots in a single planar environment.

 """

import time
import numpy as np
from planners.prm.planar.env import LOCAL_STEP
from planners.prm.planar.single_arm.prm import PRM


class CoupledPRM(PRM): 
    def __init__(self, environment) -> None:
        super().__init__(environment, local_step=LOCAL_STEP)
        self.config_space = self.create_composite_config_space(environment.robot_models) # composite configuration space

        # self.n_knn = n_knn
        # self.max_edge_len = max_edge_len
    
    # methods for coupled PRM
    def create_composite_config_space(self, robot_models):
        """Creates a composite configuration space by concatenating the config_space of all robots."""
        composite_space = []
        for model in robot_models:
            composite_space.extend(model.config_space)
        return composite_space
    
    def merge_configs(self, configs):
        """Merges multiple configurations into a single higher-dimensional configuration."""
        merged_config = np.concatenate([np.array(config) for config in configs])
        return np.array(merged_config)
    
    def split_config(self, config):
        """Splits a higher-dimensional configuration into individual robot configurations."""
        split_configs = []
        offset = 0
        for robot in self.robot_models:
            dim = len(robot.config_space)
            split_configs.append(np.array(config[offset:offset+dim]))
            offset += dim
        return split_configs
    
    # building roadmap    
    def build_roadmap_n(self, n):
        raise NotImplementedError()
    
    def build_roadmap_time(self, t):
        raise NotImplementedError()
    
    def add_n_samples(self, n):
        raise NotImplementedError()
    
    def add_build_period(self, t):
        raise NotImplementedError()

    def update_roadmap(self): 
        raise NotImplementedError()

    # distance metric
    def distance(self, q1, q2):
        raise NotImplementedError()

    # distance metric
    def specific_distance(self, q1, q2):
        split_configs1 = self.split_config(q1)
        split_configs2 = self.split_config(q2)
        dist = 0
        for i, robot in enumerate(self.robot_models):
            pos1 = robot.forward_kinematics(split_configs1[i])
            pos2 = robot.forward_kinematics(split_configs2[i])
            dist += robot.distance(pos1, pos2)
        return dist
    
    # local planner 
    def is_collision_free_sample(self, sample):
        for i, robot1 in enumerate(self.robot_models): 
            config1 = sample[i*robot1.n_links:(i+1)*robot1.n_links]
            # within map bounds
            if not self.robot_in_env_bounds(config1, robot1):
                return False
            # self collisions
            if self.robot_self_collision(config1, robot1):
                return False
            # collisions with other robots
            for j, robot2 in enumerate(self.robot_models[i+1:]): 
                # config1 = sample[i*robot1.n_links:(i+1)*robot1.n_links]
                config2 = sample[(i+j+1)*robot2.n_links:(i+j+2)*robot2.n_links]
                sample12 = self.merge_configs([config1, config2])
                if self.robot_robot_collision(sample12, robot1, robot2):
                    return False
            for obstacle in self.obstacles:
                # sample1 = sample[i*robot1.n_links:(i+1)*robot1.n_links]
                if self.robot_obstacle_collision(config1, robot1, obstacle):
                    return False
        return True
    
    def robot_robot_collision(self, sample, robot1, robot2):
        sample1, sample2 = self.split_config(sample)
        robot1.set_pose(sample1)
        robot2.set_pose(sample2)
        return self.env.robot_robot_collision(robot1, robot2)
    
    # Querying roadmap
    def query(self):
        path = []
        paths = {}

        start_config = np.array([])
        goal_config = np.array([])
        for agent in self.agents:
            # find paths from nearest node to start and nearest node to goal 
            start_config = np.append(start_config, agent['start'])
            goal_config = np.append(goal_config, agent['goal'])

        if not self.is_collision_free_sample(start_config): 
            print("Start configuration is in collision.")
            return paths
        if not self.is_collision_free_sample(goal_config):
            print("Goal configuration is in collision.")
            return paths
        
        # Find the nearest roadmap nodes to the start and goal configurations
        start_node = self.find_nearest_roadmap_node(start_config)
        goal_node = self.find_nearest_roadmap_node(goal_config)

        # # Use Dijkstra to find a path between the nearest start and goal nodes
        # dijkstra_start_time = time.perf_counter()
        # d_path, d_distance = self.dijkstra_search(start_node.id, goal_node.id)
        # dijkstra_duration = time.perf_counter() - dijkstra_start_time
        # print(f"Dijkstra: Composite path in {dijkstra_duration:.6f} seconds with distance {d_distance:.2f}")
        
        # Use A* to find a path between the nearest start and goal nodes
        a_star_start_time = time.perf_counter()
        a_path, a_distance = self.a_star_search(start_node.id, goal_node.id)
        a_star_duration = time.perf_counter() - a_star_start_time
        print(f"A*: Composite path in {a_star_duration:.6f} seconds with distance {a_distance:.2f}")
        
        # Store the path for the agent
        path = a_path
        for config in path:
            split_configs = self.split_config(self.node_id_config_dict[config])
            for i, agent in enumerate(self.agents):
                if agent['name'] not in paths:
                    paths[agent['name']] = []
                paths[agent['name']].append(tuple(split_configs[i]))
        return paths

    # compute
    def compute_combined_avg_degree(self):
        neighbor_sum = sum([len(neighbors) for neighbors in self.roadmap.values()])
        nodes_sum = len(self.roadmap)
        return neighbor_sum / nodes_sum
    
    def compute_combined_nr_nodes(self):
        return len(self.nodes)
    
    def compute_nr_edges(self, robot_id):
        return sum([len(neighbors) for neighbors in self.roadmaps[robot_id].values()]) / 2
    
    def compute_combined_nr_edges(self):
        return sum([len(neighbors) for neighbors in self.roadmap.values()]) / 2
