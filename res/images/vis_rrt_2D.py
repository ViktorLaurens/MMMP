import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
import random

from utils.planner_utils import Interval

class RRT:
    def __init__(self, start, goal, cspace, obstacles=[], max_iter=1000, step_size=1.0, goal_sample_rate=0.05):
        self.start = Node(start, None)
        self.goal = Node(goal, None)
        self.cspace = cspace
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.tree = [self.start]

    def plan(self):
        for i in range(self.max_iter):
            rnd_point = self.sample()
            nearest_node = self.nearest_node(rnd_point)
            new_node = self.steer(nearest_node, rnd_point)

            if not self.collision_free(nearest_node, new_node):
                continue

            self.tree.append(new_node)

            if self.is_goal_reached(new_node):
                return self.generate_path(new_node)
        
        return None  # If no path found within max_iter

    def sample(self):
        if random.random() > self.goal_sample_rate:
            return np.array([random.uniform(interval.lower, interval.upper) for interval in self.cspace])
        else:
            return self.goal.position

    def nearest_node(self, point):
        return min(self.tree, key=lambda node: np.linalg.norm(node.position - point))

    def steer(self, from_node, to_point):
        direction = to_point - from_node.position
        distance = np.linalg.norm(direction)
        step = self.step_size if distance > self.step_size else distance
        new_position = from_node.position + (direction / distance) * step
        return Node(new_position, from_node)

    def collision_free(self, from_node, to_node):
        for obstacle in self.obstacles:
            if obstacle.intersects(from_node.position, to_node.position):
                return False
        return True

    def is_goal_reached(self, node):
        return np.linalg.norm(node.position - self.goal.position) < self.step_size

    def generate_path(self, node):
        path = []
        while node is not None:
            path.append(node.position)
            node = node.parent
        return path[::-1]

    def visualize(self, path=None):
        fig, ax = plt.subplots()

        for node in self.tree:
            if node.parent is not None:
                ax.plot([node.position[0], node.parent.position[0]], [node.position[1], node.parent.position[1]], 'k-', lw=0.5)

        for obstacle in self.obstacles:
            obstacle.plot(ax)

        if path is not None:
            path = np.array(path)
            ax.plot(path[:, 0], path[:, 1], 'r-', linewidth=2)

        ax.plot(self.start.position[0], self.start.position[1], 'ro')
        ax.plot(self.goal.position[0], self.goal.position[1], 'go')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xticks([])
        ax.set_yticks([])
        plt.show()

class Node:
    def __init__(self, position, parent):
        self.position = np.array(position)
        self.parent = parent

class Obstacle:
    def intersects(self, start, end):
        pass  # To be implemented by subclasses

class RectangleObstacle(Obstacle):
    def __init__(self, bottom_left, width, height):
        self.bottom_left = np.array(bottom_left)
        self.width = width
        self.height = height

    def intersects(self, start, end):
        # Simple bounding box collision check
        if start[0] > end[0]:
            start, end = end, start
        
        x1, y1 = start
        x2, y2 = end
        
        left = self.bottom_left[0]
        right = self.bottom_left[0] + self.width
        bottom = self.bottom_left[1]
        top = self.bottom_left[1] + self.height
        
        if x1 < right and x2 > left and y1 < top and y2 > bottom:
            return True  # Simple overlap check
        return False

    def plot(self, ax):
        rect = Rectangle(self.bottom_left, self.width, self.height, edgecolor='black', facecolor='gray')
        ax.add_patch(rect)

def main(): 
    # Define RRT planning parameters
    start = [-9, -9]
    goal = [9, 9]
    cspace = [Interval(-10, 10), Interval(-10, 10)]
    obstacles = [RectangleObstacle([-5, -5], 3, 3)]  # Add more obstacles as needed

    # Initialize and run RRT
    rrt = RRT(start, goal, cspace, obstacles)
    path = rrt.plan()
    rrt.visualize(path)

if __name__ == '__main__':
    main()