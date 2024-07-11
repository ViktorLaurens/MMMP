from collections import deque
from copy import deepcopy
import datetime
from itertools import combinations
import math
import time
from matplotlib.patches import Circle, Polygon, Rectangle
# from shapes import Rectangle, Circle, Polygon
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# TYPE = 'nn' # 'distance' or 'nn'
LOCAL_STEP = 0.05  # [rad] Step size for local planner

# constants
# INF = float("inf")

class Environment:
    def __init__(self, map_dimensions, agents, obstacles):
        self.map_dimensions = map_dimensions
        self.map_center = [0, 0]
        self.map_lower_bounds = [-map_dimensions[0]/2, -map_dimensions[1]/2]
        self.map_upper_bounds = [map_dimensions[0]/2, map_dimensions[1]/2]
        self.map_lower_x_bound = self.map_lower_bounds[0]
        self.map_upper_x_bound = self.map_upper_bounds[0]
        self.map_lower_y_bound = self.map_lower_bounds[1]
        self.map_upper_y_bound = self.map_upper_bounds[1]

        self.agents = agents
        self.robot_models = [agent["model"] for agent in agents]
        self.obstacles = obstacles

        # check that obstcales are within map bounds
        for obstacle in obstacles:
            if not self.obstacle_in_env_bounds(obstacle):
                raise ValueError("Obstacle is out of environment bounds.")

        # Initialize robot positions
        for i, agent in enumerate(agents):
            this_model = self.robot_models[i]
            this_model.set_pose(agent["start"])
            # check for collisions
            if self.robot_collision(this_model):
                raise ValueError(f"Agent {agent['name']} start configuration is in collision.")

    # Obstacles in bounds check
    def obstacle_in_env_bounds(self, obstacle):
        if isinstance(obstacle, Rectangle):
            return self.point_in_env_bounds(obstacle.get_bbox().get_points()[0]) and self.point_in_env_bounds(obstacle.get_bbox().get_points()[1])
        elif isinstance(obstacle, Circle):
            return self.point_in_env_bounds(obstacle.center + np.array([-obstacle.radius, 0])) \
                and self.point_in_env_bounds(obstacle.center + np.array([0, -obstacle.radius]))\
                and self.point_in_env_bounds(obstacle.center + np.array([obstacle.radius, 0])) \
                and self.point_in_env_bounds(obstacle.center + np.array([0, obstacle.radius]))

    # robot collision is due to: self collision, env_bounds, other robots and obstacles
    def robot_collision(self, robot):
        if not self.robot_in_env_bounds(robot):
            print("Robot out of environment bounds")
            return True, "Robot out of environment bounds"
        # if self.robot_self_collision(robot):
        #     print("Robot self collision")
        #     return True, "Robot self collision"
        for obstacle in self.obstacles:
            if self.robot_obstacle_collision(robot, obstacle):
                print("Robot obstacle collision")
                return True, "Robot obstacle collision"
        for other_robot in self.robot_models:
            if other_robot != robot and self.robot_robot_collision(robot, other_robot):
                print("Robot robot collision")
                return True, "Robot robot collision"
        return False, "No collision"

    # robot robot collision
    def robot_robot_collision(self, robot1, robot2):
        # check if links intersect
        for i in range(robot1.n_links):
            if i == 0:
                prev_pos = robot1.base_pos
            else:
                prev_pos = robot1.pos[i-1]
            for j in range(robot2.n_links):
                if j == 0:
                    prev_pos2 = robot2.base_pos
                else:
                    prev_pos2 = robot2.pos[j-1]
                if self.link_link_collision(robot1.pos[i], prev_pos, robot2.pos[j], prev_pos2):
                    return True
                
        # check if robot1 joint comes close to robot2 link
        for i in range(robot1.n_links):
            joint_i_pos = robot1.pos[i]
            for j in range(robot2.n_links):
                if j == 0:
                    link_j_lower_pos = robot2.base_pos
                else:
                    link_j_lower_pos = robot2.pos[j-1]
                link_j_upper_pos = robot2.pos[j]
                if self.joint_link_collision(joint_i_pos, link_j_lower_pos, link_j_upper_pos):
                    return True
                
        # check if robot2 joint comes close to robot1 link
        for i in range(robot2.n_links):
            joint_i_pos = robot2.pos[i]
            for j in range(robot1.n_links):
                if j == 0:
                    link_j_lower_pos = robot1.base_pos
                else:
                    link_j_lower_pos = robot1.pos[j-1]
                link_j_upper_pos = robot1.pos[j]
                if self.joint_link_collision(joint_i_pos, link_j_lower_pos, link_j_upper_pos):
                    return True
                
        return False
    
    def joint_link_collision(self, joint_pos, link_lower_pos, link_upper_pos):
        # calculate projection of joint on link
        link_vector = link_upper_pos - link_lower_pos
        joint_vector = joint_pos - link_lower_pos
        proj = np.dot(joint_vector, link_vector) / np.dot(link_vector, link_vector)
        # check if projection is within link bounds
        if proj < 0 or proj > 1:
            return False
        else: 
            # calculate distance between joint and link
            distance = np.linalg.norm(np.cross(link_vector, joint_vector)) / np.linalg.norm(link_vector)
            return distance <= 0.5
    
    def link_link_collision(self, p1, q1, p2, q2):
        # Check if the line segments intersect
        def ccw(a, b, c):
            # Calculate the cross product of vectors (b - a) and (c - a)
            cross_product = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            return cross_product > 0
        # if self.collinear(p1, q1, p2, q2):
        #     return min(p1[0], q1[0]) <= max(p2[0], q2[0]) and min(p2[0], q2[0]) <= max(p1[0], q1[0])
        # else: 
        return ccw(p1, q1, p2) != ccw(p1, q1, q2) and ccw(p2, q2, p1) != ccw(p2, q2, q1)

    # def collinear(self, p1, q1, p2, q2):
    #     # Check if the line segments are collinear
    #     return (q1[1] - p1[1]) * (p2[0] - q2[0]) == (p2[1] - q2[1]) * (q1[0] - p1[0])
    
    #  robot obstacle collision
    def robot_obstacle_collision(self, robot, obstacle):
        for i in range(robot.n_links):
            if i == 0:
                prev_pos = robot.base_pos
            else:
                prev_pos = robot.pos[i-1]
            if self.link_obstacle_collision(robot.pos[i], prev_pos, obstacle):
                return True
        return False
    
    def link_obstacle_collision(self, p, q, obstacle):
        # Check if the line segment intersects with the obstacle
        return self.line_intersects_obs(p, q, obstacle)
    
    def line_intersects_obs(self, p, q, obstacle):
        if isinstance(obstacle, Rectangle):
            return self.line_intersects_rect(p, q, obstacle)
        elif isinstance(obstacle, Circle):
            return self.line_intersects_circle(p, q, obstacle)
    
    def line_intersects_rect(self, p, q, rect):
        # Unpack rectangle info
        rect_x, rect_y, rect_width, rect_height = rect.get_bbox().bounds
        rect_vertices = [
            np.array([rect_x, rect_y]),
            np.array([rect_x + rect_width, rect_y]),
            np.array([rect_x + rect_width, rect_y + rect_height]),
            np.array([rect_x, rect_y + rect_height])
        ]

        rect_edges = [
            (rect_vertices[0], rect_vertices[1]),
            (rect_vertices[1], rect_vertices[2]),
            (rect_vertices[2], rect_vertices[3]),
            (rect_vertices[3], rect_vertices[0])
        ]

        for edge_start, edge_end in rect_edges:
            if self.link_link_collision(p, q, edge_start, edge_end):
                return True
        return False
    
    def line_intersects_circle(self, p, q, circle):
        # Unpack circle info
        circle_center = circle.center
        circle_radius = circle.radius

        # Calculate the vector from p to the circle center
        v = np.array(circle_center) - p

        # Calculate the projection of v onto the line segment
        t = np.dot(v, q - p) / np.dot(q - p, q - p)

        # Calculate the closest point on the line segment to the circle center
        closest_point = p + t * (q - p)

        # Check if the closest point is within the line segment and within the circle
        return np.linalg.norm(closest_point - np.array(circle_center)) <= circle_radius and 0 <= t <= 1

    # Robot self collision
    def robot_self_collision(self, robot):
        for i in range(robot.n_links):
            if i == 0:
                prev_pos = robot.base_pos
            else:
                prev_pos = robot.pos[i-1]
            for j in range(i+2, robot.n_links):
                if self.link_link_collision(robot.pos[i], prev_pos, robot.pos[j], robot.pos[j-1]):
                    return True
        return False
    
    # Environment bounds
    def robot_in_env_bounds(self, robot):
        if not self.point_in_env_bounds(robot.base_pos):
            return False 
        for pos in robot.pos:
            if not self.point_in_env_bounds(pos):
                return False
        return True
    
    def point_in_env_bounds(self, point):
        return self.map_lower_x_bound <= point[0] <= self.map_upper_x_bound and self.map_lower_y_bound <= point[1] <= self.map_upper_y_bound

    # Visualize
    def visualize(self):
        fig, ax = plt.subplots() 

        for obstacle in self.obstacles:
            ax.add_patch(obstacle)
        
        for robot in self.robot_models:
            robot.plot(ax)
        
        ax.set_aspect('equal', adjustable='box')

        # Set the limits of the plot according to the map dimensions
        ax.set_xlim([self.map_lower_x_bound, self.map_upper_x_bound])
        ax.set_ylim([self.map_lower_y_bound, self.map_upper_y_bound])

        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks
        plt.show()

    #  Execute motion
    def execute_motion(self, paths, timestep=0.1, local_step=LOCAL_STEP):
        fig, ax = plt.subplots() 

        ax.set_aspect('equal', adjustable='box')
        
        # Set the limits of the plot according to the map dimensions
        ax.set_xlim([-self.map_dimensions[0]/2, self.map_dimensions[0]/2])
        ax.set_ylim([-self.map_dimensions[1]/2, self.map_dimensions[1]/2])

        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks

        for obstacle in self.obstacles:
            ax.add_patch(obstacle)

        # Plot start configs
        for i, robot_model in enumerate(self.robot_models):
            robot_model.set_pose(self.agents[i]["start"])
            robot_model.plot(ax)

        plt.draw()
        plt.pause(timestep)  # Pause for the timestep
        # plt.show()  # Show the plot after initializing

        # Plot intermediate nodes
        n_nodes = max([len(path) for path in paths.values()])
        for i in range(n_nodes):
            ax.clear()  # Clear the plot for each timestep
            ax.set_aspect('equal', adjustable='box')
        
            # Set the limits of the plot according to the map dimensions
            ax.set_xlim([-self.map_dimensions[0]/2, self.map_dimensions[0]/2])
            ax.set_ylim([-self.map_dimensions[1]/2, self.map_dimensions[1]/2])

            ax.set_xticks([])  # Disable x-axis ticks
            ax.set_yticks([])  # Disable y-axis ticks
            for obstacle in self.obstacles:
                ax.add_patch(obstacle)  # Add the obstacles back in
            for agent in self.agents:
                if agent["name"] in paths:
                    path = paths[agent["name"]]
                    if i < len(path):
                        agent["model"].set_pose(path[i])
                        agent["model"].plot(ax)
            plt.draw()  # Update the plot
            plt.pause(timestep)  # Pause for the timestep

        # Plot goal configs
        ax.clear()  # Clear the plot for each timestep
        ax.set_aspect('equal', adjustable='box')
    
        # Set the limits of the plot according to the map dimensions
        ax.set_xlim([-self.map_dimensions[0]/2, self.map_dimensions[0]/2])
        ax.set_ylim([-self.map_dimensions[1]/2, self.map_dimensions[1]/2])

        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks
        
        for obstacle in self.obstacles:
            ax.add_patch(obstacle)
        
        for i, robot_model in enumerate(self.robot_models):
            robot_model.set_pose(self.agents[i]["goal"])
            robot_model.plot(ax)
        
        plt.draw()
        plt.show()

    def execute_interpolated_motion(self, paths, timestep=0.01, local_step=LOCAL_STEP):
        fig, ax = plt.subplots() 

        ax.set_aspect('equal', adjustable='box')
        
        # Set the limits of the plot according to the map dimensions
        ax.set_xlim([-self.map_dimensions[0]/2, self.map_dimensions[0]/2])
        ax.set_ylim([-self.map_dimensions[1]/2, self.map_dimensions[1]/2])

        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks

        for obstacle in self.obstacles:
            ax.add_patch(obstacle)

        # Plot start configs
        for i, robot_model in enumerate(self.robot_models):
            robot_model.set_pose(self.agents[i]["start"])
            robot_model.plot(ax)

        plt.draw()
        plt.pause(timestep)  # Pause for the timestep
        # plt.show()  # Show the plot after initializing

        # Plot intermediate nodes
        max_distance = 0
        for path in paths.values():
            distance = 0
            for i in range(len(path)-1):
                distance += np.linalg.norm(np.array(path[i]) - np.array(path[i+1]))
            if distance > max_distance: 
                max_distance = distance

        interpolated_paths = {}
        for agent_name, path in paths.items():
            interpolated_paths.update({agent_name: []})
            interpolated_path = []
            for i in range(len(path)-1):
                distance = np.linalg.norm(np.array(path[i]) - np.array(path[i+1]))
                n_steps = math.ceil(distance/local_step)
                for j in range(n_steps):
                    interpolated_path.append(np.array(path[i]) + j/n_steps * (np.array(path[i+1]) - np.array(path[i])))
            interpolated_paths[agent_name] = interpolated_path
            
        n_nodes = max([len(path) for path in interpolated_paths.values()])
        for i in range(n_nodes):
            ax.clear()  # Clear the plot for each timestep
            ax.set_aspect('equal', adjustable='box')
        
            # Set the limits of the plot according to the map dimensions
            ax.set_xlim([-self.map_dimensions[0]/2, self.map_dimensions[0]/2])
            ax.set_ylim([-self.map_dimensions[1]/2, self.map_dimensions[1]/2])

            ax.set_xticks([])  # Disable x-axis ticks
            ax.set_yticks([])  # Disable y-axis ticks
            for obstacle in self.obstacles:
                ax.add_patch(obstacle)  # Add the obstacles back in
            for agent in self.agents:
                if agent["name"] in interpolated_paths:
                    path = interpolated_paths[agent["name"]]
                    if i < len(path):
                        agent["model"].set_pose(path[i])
                        agent["model"].plot(ax)
            plt.draw()  # Update the plot
            plt.pause(timestep)  # Pause for the timestep

        # Plot goal configs
        ax.clear()  # Clear the plot for each timestep
        ax.set_aspect('equal', adjustable='box')
    
        # Set the limits of the plot according to the map dimensions
        ax.set_xlim([-self.map_dimensions[0]/2, self.map_dimensions[0]/2])
        ax.set_ylim([-self.map_dimensions[1]/2, self.map_dimensions[1]/2])

        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks
        
        for obstacle in self.obstacles:
            ax.add_patch(obstacle)
        
        for i, robot_model in enumerate(self.robot_models):
            robot_model.set_pose(self.agents[i]["goal"])
            robot_model.plot(ax)
        
        plt.draw()
        plt.show()

    def execute_decoupled_interpolated_motion(self, paths, plot_timestep=0.01):
        if not paths:
            print("No paths to visualize.")
            return
        
        fig, ax = plt.subplots() 

        ax.set_aspect('equal', adjustable='box')
        
        # Set the limits of the plot according to the map dimensions
        ax.set_xlim([-self.map_dimensions[0]/2, self.map_dimensions[0]/2])
        ax.set_ylim([-self.map_dimensions[1]/2, self.map_dimensions[1]/2])

        ax.set_xticks([])  # Disable x-axis ticks
        ax.set_yticks([])  # Disable y-axis ticks

        for obstacle in self.obstacles:
            ax.add_patch(obstacle)

        # Plot start configs
        for i, robot_model in enumerate(self.robot_models):
            robot_model.set_pose(self.agents[i]["start"])
            robot_model.plot(ax)

        plt.draw()
        plt.pause(plot_timestep)  # Pause for the timestep
        # plt.show()  # Show the plot after initializing

        # Get the maximum time index across all paths
        times = {robot_id: max(path.keys()) for robot_id, path in paths.items()}
        max_time = max(times.values())

        # Generate a list of time indices from 0 to max_t_idx with step size timestep
        timestep = np.round(sorted(paths[0].keys())[1] - sorted(paths[0].keys())[0], 1)      
        t_indices = np.arange(0, max_time + timestep, timestep)

        for t_idx in t_indices:
            # ax.clear()
            # ax.set_aspect('equal', adjustable='box')    
            # ax.set_xlim([-self.map_dimensions[0]/2, self.map_dimensions[0]/2])
            # ax.set_ylim([-self.map_dimensions[1]/2, self.map_dimensions[1]/2])
            # ax.set_xticks([])  # Disable x-axis ticks
            # ax.set_yticks([])  # Disable y-axis ticks
            for obstacle in self.obstacles:
                ax.add_patch(obstacle)
            for r_id, path in paths.items():
                # Find the key in the path that is closest to the current time index
                closest_key = min(path.keys(), key=lambda x: abs(x - t_idx))
                self.robot_models[r_id].set_pose(path[closest_key])
                self.robot_models[r_id].plot(ax)
                # if len(str(closest_key)) > 3: 
                #     self.robot_models[r_id].plot_goal(ax)
                # else: 
                #     self.robot_models[r_id].plot(ax)
            plt.draw()
            plt.pause(timestep)
            if t_idx >= max_time:
                for i, robot_model in enumerate(self.robot_models):
                    robot_model.set_pose(self.agents[i]["start"])
                    robot_model.plot_start(ax)
                    robot_model.set_pose(self.agents[i]["goal"])
                    robot_model.plot_goal(ax)
                plt.draw()
                plt.show()
                break
        return