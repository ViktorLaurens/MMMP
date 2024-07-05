import pybullet as p

from utils.pb_body_utils import get_base_name
from utils.pb_conf_utils import CLIENT
from utils.pb_joint_utils import get_joint_info, get_joints

"""
LINKS Utilities

This module provides utility functions and definitions for handling links within simulations or visualizations.
"""
# ---------------------
# LINKS variables for link identification and mass properties
BASE_LINK = -1
STATIC_MASS = 0
# ---------------------

# ---------------------
# LINKS functions for getting info from links
def get_link_name(body, link):
    if link == BASE_LINK:
        return get_base_name(body)
    return get_joint_info(body, link).linkName.decode('UTF-8')

def link_from_name(body, name):
    if name == get_base_name(body):
        return BASE_LINK
    for link in get_joints(body):
        if get_link_name(body, link) == name:
            return link
    raise ValueError(body, name)

def get_link_parent(body, link):
    if link == BASE_LINK:
        return None
    return get_joint_info(body, link).parentIndex

parent_link_from_joint = get_link_parent

def child_link_from_joint(joint):
    # note that link index == joint index
    link = joint
    return link

def parent_joint_from_link(link):
    # note that link index == joint index
    joint = link
    return joint

def get_link_ancestors(body, link):
    parent = get_link_parent(body, link)
    if parent is None:
        return []
    return get_link_ancestors(body, parent) + [parent]

def get_ordered_ancestors(robot, link):
    return get_link_ancestors(robot, link)[1:] + [link]

def get_num_links(body):
    return p.getNumJoints(body, physicsClientId=CLIENT)

def get_links(body):
    return list(range(get_num_links(body))) # Does not include BASE_LINK

# def child_link_from_joint(joint):
#     # note that link index == joint index
#     link = joint
#     return link

# def get_link_parent(body, link):
#     if link == BASE_LINK:
#         return None
#     return get_joint_info(body, link).parentIndex

# parent_link_from_joint = get_link_parent

# def get_all_links(body):
#     # TODO: deprecate get_links
#     return [BASE_LINK] + list(get_links(body))

# def get_link_name(body, link):
#     if link == BASE_LINK:
#         return get_base_name(body)
#     return get_joint_info(body, link).linkName.decode('UTF-8')

# def get_joint_pair(body, joint):
#     child = child_link_from_joint(joint)
#     parent = parent_link_from_joint(body, joint)
#     return child, parent
# ---------------------