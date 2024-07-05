import pybullet as p

from collections import namedtuple
from utils.pb_conf_utils import CLIENT
from utils.pb_link_utils import get_ordered_ancestors, link_from_name, parent_joint_from_link, parent_link_from_joint
from utils.pb_joint_utils import joints_from_names, prune_fixed_joints


"""
IK Utilities

This module provides utility functions and definitions for handling inverse kinematics within simulations or visualizations.
"""
# ---------------------
# IK variables
IK_info = namedtuple('IK_info', ['base_link', 'ee_link', 'free_joints'])
# ---------------------

# ---------------------
# Calculating IK
def calculate_ik(robotId, eeIndex, targetPose): 
    targetPosition, targetOrientation = targetPose
    return p.calculateInverseKinematics(robotId, eeIndex, targetPosition, targetOrientation)

def calculate_arm_ik(robotId, eeIndex, targetPose): 
    return calculate_ik(robotId, eeIndex, targetPose)[:-2]
# ---------------------

# ---------------------
# GETTERS and SETTERS
def get_ik_joints(robot, ik_info, tool_link):
    # Get joints between base and ee
    base_link = link_from_name(robot, ik_info.base_link)
    ee_link = link_from_name(robot, ik_info.ee_link)
    ee_ancestors = get_ordered_ancestors(robot, ee_link)
    tool_ancestors = get_ordered_ancestors(robot, tool_link)
    # Ensure only fixed joints between ee and tool
    assert prune_fixed_joints(robot, ee_ancestors) == prune_fixed_joints(robot, tool_ancestors)
    # Find first joint
    first_joint = None
    for link in tool_ancestors:
        parent_joint = parent_joint_from_link(link)
        parent_link = parent_link_from_joint(robot, parent_joint)
        if parent_link == base_link:
            first_joint = parent_joint
            break  # Stop searching once the first joint is found
    # Ensure that a first joint was found
    assert first_joint is not None, "No joint found connecting the tool to the base link."
    ik_joints = prune_fixed_joints(robot, ee_ancestors[ee_ancestors.index(first_joint):])
    free_joints = joints_from_names(robot, ik_info.free_joints)
    assert set(free_joints) <= set(ik_joints)
    assert len(ik_joints) == 6 + len(free_joints)
    return ik_joints
# ---------------------