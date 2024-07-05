import numpy as np
import pybullet as p

"""
GEOMETRY Utilities

This module provides utility functions and definitions for handling geometry within simulations or visualizations.
"""

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler)

def euler_from_quat(quat):
    return p.getEulerFromQuaternion(quat)

def convex_combination(x, y, w=0.5):
    return (1-w)*np.array(x) + w*np.array(y)