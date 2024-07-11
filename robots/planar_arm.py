"""

This is a simple planar arm class to be used together with 
planners from 'planar' directories. 

"""

import numpy as np

from utils.planner_utils import Interval

class PlanarArm:
    def __init__(self, links, base_pos=np.array([0, 0])):
        self.links = links
        self.base_pos = base_pos
        self.n_links = links.shape[0]
        self.config_space = [Interval(lower=-np.pi, upper=np.pi) for _ in range(self.n_links)]

        self.q = np.zeros(self.n_links)
        self.q_d = np.zeros(self.n_links)
        self.pos = np.zeros((self.n_links, 2))

    def forward_kinematics(self, q):
        """
        Calculates links positions given joint angles

        Parameters
        ----------
        q : numpy.ndarray
            (n_links,) array of angles in radians
        """

        pos = np.zeros((self.n_links, 2))
        pos[0, :] = self.base_pos + np.array([[self.links[0] * np.cos(q[0]), self.links[0] * np.sin(q[0])]])
        for i in range(1, self.n_links):
            delta_pos = np.array([self.links[i] * np.cos(np.sum(q[0:i+1])),
                                  self.links[i] * np.sin(np.sum(q[0:i+1]))])
            pos[i, :] = pos[i - 1, :] + delta_pos

        return pos
    
    def set_pose(self, q):
        """
        Set the joint angles of the arm

        Parameters
        ----------
        q : numpy.ndarray
            (n_links,) array of angles in radians
        """
        self.q = q
        self.pos = self.forward_kinematics(q)

    def distance(self, pos1, pos2):
        """
        Calculate specific distance metric 
        """
        distance = 0
        for i in range(self.n_links):
            distance += np.linalg.norm(pos1[i] - pos2[i])**2
        return distance
        
    def plot(self, ax, arm_width=1, joint_size=5):
        ax.scatter(self.base_pos[0], self.base_pos[1], c='k', zorder=10, s=10)
        ax.plot([self.base_pos[0], self.pos[0, 0]], [self.base_pos[1], self.pos[0, 1]], c='orange', linewidth=arm_width)
        
        for i in range(self.n_links):
            ax.scatter(self.pos[i, 0], self.pos[i, 1], c='blue', zorder=10, s=joint_size)
            if i >= 1:
                ax.plot([self.pos[i - 1, 0], self.pos[i, 0]], [self.pos[i - 1, 1], self.pos[i, 1]], c='orange', linewidth=arm_width)

    def plot_start(self, ax, arm_width=1, joint_size=5):
        ax.scatter(self.base_pos[0], self.base_pos[1], c='k', zorder=10, s=10)
        ax.plot([self.base_pos[0], self.pos[0, 0]], [self.base_pos[1], self.pos[0, 1]], c='red', linewidth=arm_width)
        
        for i in range(self.n_links):
            ax.scatter(self.pos[i, 0], self.pos[i, 1], c='red', zorder=10, s=joint_size)
            if i >= 1:
                ax.plot([self.pos[i - 1, 0], self.pos[i, 0]], [self.pos[i - 1, 1], self.pos[i, 1]], c='red', linewidth=arm_width)

    def plot_goal(self, ax, arm_width=1, joint_size=5):
        ax.scatter(self.base_pos[0], self.base_pos[1], c='k', zorder=10, s=10)
        ax.plot([self.base_pos[0], self.pos[0, 0]], [self.base_pos[1], self.pos[0, 1]], c='green', linewidth=arm_width)
        
        for i in range(self.n_links):
            ax.scatter(self.pos[i, 0], self.pos[i, 1], c='green', zorder=10, s=joint_size)
            if i >= 1:
                ax.plot([self.pos[i - 1, 0], self.pos[i, 0]], [self.pos[i - 1, 1], self.pos[i, 1]], c='green', linewidth=arm_width)

    # def plot(self, pos, ax, arm_width=1, joint_size=5):
    #     """
    #     Plot arms and links given their locations

    #     Parameters
    #     ----------
    #     pos : numpy.ndarray
    #         (N, 2) matrix of coordinates

    #     ax
    #         Matplotlib axis object
    #     """
    #     ax.scatter(self.base_pos[0], self.base_pos[1], c='k', zorder=10, s=100)
    #     ax.plot([self.base_pos[0], pos[0, 0]], [self.base_pos[1], pos[0, 1]], c='orange', linewidth=arm_width)
        
    #     for i in range(self.n_links):
    #         ax.scatter(pos[i, 0], pos[i, 1], c='blue', zorder=10, s=joint_size)
    #         if i >= 1:
    #             ax.plot([pos[i - 1, 0], pos[i, 0]], [pos[i - 1, 1], pos[i, 1]], c='orange', linewidth=arm_width)
        
    # def visualize(self, q=None):
    #     fig, ax = plt.subplots() 

    #     if q is not None:
    #         pos = self.forward_kinematics(q)
    #     else:
    #         pos = self.pos
    #     self.plot(pos, ax)
    #     plt.show()
