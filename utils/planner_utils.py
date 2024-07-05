import numpy as np
import pybullet as p
import math

from collections import namedtuple
from utils.pb_joint_utils import get_joint_info, get_joint_type, set_joint_positions

"""
MOTION PLANNING Utilities

This module provides utility functions and definitions for motion planning within simulations or visualizations.
"""
# ---------------------
# SAMPLING
# ---------------------
# Named tuple for defining intervals
Interval = namedtuple('Interval', ['lower', 'upper'])

# Predefined constants for limit intervals
UNIT_LIMITS = Interval(0., 1.)
CIRCULAR_LIMITS = Interval(-np.pi, np.pi)
UNBOUNDED_LIMITS = Interval(-np.inf, np.inf)

def is_circular_joint(body, joint):
    """
    Determines if a joint is circular based on its limits.
    """
    joint_info = get_joint_info(body, joint)
    return joint_info.jointType != p.JOINT_FIXED and joint_info.jointUpperLimit < joint_info.jointLowerLimit

def get_joint_limit_interval(body, joint):
    """
    Returns the limit interval for a joint. For circular joints, it returns the circular limits.
    """
    if is_circular_joint(body, joint):
        return CIRCULAR_LIMITS
    joint_info = get_joint_info(body, joint)
    return Interval(joint_info.jointLowerLimit, joint_info.jointUpperLimit)

def get_custom_joint_intervals(body, joints, custom_limits={}):
    """
    Returns a list of intervals for the provided joints, considering custom limits if specified.
    """
    intervals = []
    for joint in joints:
        if joint in custom_limits:
            intervals.append(custom_limits[joint])
        else:
            intervals.append(get_joint_limit_interval(body, joint))
    return intervals

def sample_from_interval(interval):
    """ Yield a continuous stream of samples from a given interval. """
    while True:
        yield np.random.uniform(interval.lower, interval.upper)

def create_sample_generator(body, joints, custom_limits={}):
    """
    Create a generator function that yields joint configurations within the joint limits.
    """
    intervals = get_custom_joint_intervals(body, joints, custom_limits)
    generators = [sample_from_interval(interval) for interval in intervals]

    def sample_generator():
        while True:
            # Generate a sample configuration by sampling each joint
            yield tuple(next(gen) for gen in generators)

    return sample_generator

# def create_sample_generator(body, joints, custom_limits={}):
#     """
#     Create a function that returns a new joint configuration sample within the joint limits.
#     """
#     intervals = get_custom_joint_intervals(body, joints, custom_limits)
    
#     def sample():
#         # Directly generate and return a sample configuration by sampling each joint interval
#         return tuple(np.random.uniform(interval.lower, interval.upper) for interval in intervals)

#     return sample

# ---------------------
# DISTANCE
# ---------------------
def circular_difference(theta2, theta1):
    """
    Calculate the shortest circular difference between two angles.
    """
    pi2 = np.pi * 2
    diff = (theta2 - theta1 + np.pi) % pi2 - np.pi
    return diff

def get_difference_fn(body, joints):
    """
    Return a function that calculates the differences between two joint configurations,
    considering circular joints.
    """
    circular_joints = [is_circular_joint(body, joint) for joint in joints]

    def difference(q2, q1):
        # print("q2:", q2, "q1:", q1)
        diffs = [
            circular_difference(v2, v1) if circular else v2 - v1
            for circular, v2, v1 in zip(circular_joints, q2, q1)
        ]
        return tuple(diffs)
    
    return difference

def get_default_weights(joints, weights=None):
    """
    Get default weights for the joints if custom weights are not provided.
    """
    return np.ones(len(joints)) if weights is None else weights

def get_distance_fn(body, joints, weights=None, norm=2):
    """
    Create a function to calculate the distance between two configurations of the specified joints.
    """
    weights = get_default_weights(joints, weights)
    difference_fn = get_difference_fn(body, joints)

    def distance(q1, q2):
        diffs = np.array(difference_fn(q2, q1))
        if norm == 2:
            return np.sqrt(np.dot(weights, diffs * diffs))
        else:
            return np.linalg.norm(np.multiply(weights, diffs), ord=norm)
    
    return distance

# ---------------------
# EXTENSION
# ---------------------
def wrap_interval(value, interval=UNIT_LIMITS):
    lower, upper = interval
    if (lower == -np.inf) and (+np.inf == upper):
        return value
    assert -np.inf < lower <= upper < +np.inf
    return (value - lower) % (upper - lower) + lower

def circular_interval(lower=-np.pi): # [-np.pi, np.pi]
    return Interval(lower, lower + 2*np.pi)

def wrap_angle(theta):
    return wrap_interval(theta, interval=circular_interval())

def wrap_position(body, joint, position):
    """Wrap joint position based on whether the joint is circular."""
    return wrap_angle(position) if is_circular_joint(body, joint) else position

def wrap_positions(body, joints, positions):
    """Wrap positions of multiple joints."""
    assert len(joints) == len(positions), "Joints and positions length mismatch."
    return [wrap_position(body, joint, pos) for joint, pos in zip(joints, positions)]

# def get_refine_fn(body, joints, num_steps=0):
#     """Generate intermediate configurations between two given configurations."""
#     difference_fn = get_difference_fn(body, joints)
#     num_steps = num_steps + 1
#     def fn(q1, q2):
#         """Yield refined positions between q1 and q2 over specified steps."""
#         q = q1
#         for i in range(num_steps):
#             positions = (1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q
#             q = tuple(wrap_positions(body, joints, positions))
#             yield q
#     return fn

def generate_intermediate_configs(body, joints, steps=0):
    """
    Creates a function to generate intermediate configurations between two configurations.
    This allows for a gradual transition from one configuration (q1) to another (q2) over a specified number of steps.

    Parameters:
    - body: The PyBullet body ID.
    - joints: A list of joint indices for which the configurations are defined.
    - steps: The number of intermediate steps to generate between q1 and q2.

    Returns:
    - A generator function that yields intermediate configurations between q1 and q2.
    """
    difference_fn = get_difference_fn(body, joints)  # Get the function to calculate differences considering circular joints.
    
    def interpolate_configs(q1, q2):
        """
        A generator function that yields configurations between q1 and q2, including both.
        
        Yields:
        - Intermediate configurations between q1 and q2 over 'steps + 1' steps.
        """
        for step in range(1, steps + 2):
            alpha = step / (steps + 1)  # Calculate the interpolation factor
            delta = np.array(difference_fn(q2, q1))  # Calculate the difference between q2 and q1
            interpolated = q1 + alpha * delta  # Calculate the interpolated configuration
            wrapped_config = wrap_positions(body, joints, interpolated)  # Ensure configurations are within valid limits
            yield tuple(wrapped_config)
            
    return interpolate_configs

DEFAULT_RESOLUTION = math.radians(3) # 0.05

def get_default_resolution(body, joint):
    joint_type = get_joint_type(body, joint)
    if joint_type == p.JOINT_REVOLUTE:
        return math.radians(3) # 0.05
    elif joint_type == p.JOINT_PRISMATIC:
        return 0.02
    return DEFAULT_RESOLUTION

def get_default_resolutions(body, joints, resolutions=None):
    if resolutions is not None:
        return resolutions
    return np.array([get_default_resolution(body, joint) for joint in joints])

def get_extend_fn(body, joints, resolutions=None, norm=2):
    # norm = 1, 2, INF
    resolutions = get_default_resolutions(body, joints, resolutions)
    difference_fn = get_difference_fn(body, joints)
    def fn(q1, q2):
        #steps = int(np.max(np.abs(np.divide(difference_fn(q2, q1), resolutions))))
        steps = int(np.linalg.norm(np.divide(difference_fn(q2, q1), resolutions), ord=norm))
        interpolation_fn = generate_intermediate_configs(body, joints, steps=steps)
        return interpolation_fn(q1, q2)
    return fn

# ---------------------
# COLLISION
# ---------------------

MAX_DISTANCE = 0. # 0. | 1e-3

def get_custom_limits(body, joints, custom_limits={}, circular_limits=UNBOUNDED_LIMITS):
    joint_limits = []
    for joint in joints:
        if joint in custom_limits:
            joint_limits.append(custom_limits[joint])
        elif is_circular_joint(body, joint):
            joint_limits.append(circular_limits)
        else:
            interval = get_joint_limit_interval(body, joint)
            joint_limits.append((interval.lower, interval.upper))
    return zip(*joint_limits)

def all_between(lower_limits, values, upper_limits):
    assert len(lower_limits) == len(values)
    assert len(values) == len(upper_limits)
    return np.less_equal(lower_limits, values).all() and \
           np.less_equal(values, upper_limits).all()

def get_limits_fn(body, joints, custom_limits={}, verbose=False):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits)

    def limits_fn(q):
        if not all_between(lower_limits, q, upper_limits):
            #print('Joint limits violated')
            #if verbose: print(lower_limits, q, upper_limits)
            return True
        return False
    return limits_fn

#################################

def get_collision_fn(body, joints, obstacles=[], self_collisions=True, disabled_collisions=set(),
                     custom_limits={}, max_distance=0.0):
    # limits_fn = get_limits_fn(body, joints, custom_limits=custom_limits)

    def collision_fn(q, verbose=False):
        # Check if the configuration is within joint limits
        # if not limits_fn(q):
        #     return True
        
        # Apply the joint configuration to the body
        set_joint_positions(body, joints, q)

        # # Check for self-collisions if enabled
        # if self_collisions:
        #     # Checking self-collision by comparing all links of the body with each other
        #     for link1 in range(p.getNumJoints(body)):
        #         for link2 in range(link1 + 1, p.getNumJoints(body)):
        #             # Ensure we're not checking disabled collisions
        #             if {link1, link2} not in disabled_collisions:
        #                 closest_points = p.getClosestPoints(bodyA=body, bodyB=body, distance=max_distance,
        #                                                     linkIndexA=link1, linkIndexB=link2)
        #                 if closest_points:
        #                     if verbose:
        #                         print(f"Self-collision detected between links {link1} and {link2}")
        #                     return True

        # Check for collisions with obstacles
        for obstacle in obstacles:
            closest_points = p.getClosestPoints(bodyA=body, bodyB=obstacle, distance=max_distance)
            if closest_points:
                if verbose:
                    print(f"Collision detected with obstacle {obstacle}")
                return True
            
        # No collisions detected
        return False
    
    return collision_fn

def check_initial_end(start_conf, end_conf, collision_fn, verbose=True):
    if collision_fn(start_conf, verbose=verbose):
        print('Warning: initial configuration is in collision')
        return False
    if collision_fn(end_conf, verbose=verbose):
        print('Warning: end configuration is in collision')
        return False
    return True