import pybullet as p

from collections import namedtuple
from utils.pb_conf_utils import CLIENT, INFO_FROM_BODY
from utils.pb_geom_utils import euler_from_quat, quat_from_euler

"""
BODIES Utilities

This module provides utility functions and definitions for handling bodies within simulations or visualizations.
"""
# ---------------------
# BODIES variables
BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])
# ---------------------

# ---------------------
# BODIES functions
def get_body_info(body):
    return BodyInfo(*p.getBodyInfo(body, physicsClientId=CLIENT))

# def get_bodies():
#     return [p.getBodyUniqueId(i, physicsClientId=CLIENT) for i in range(p.getNumBodies(physicsClientId=CLIENT))]

def get_base_name(body):
    return get_body_info(body).base_name.decode(encoding='UTF-8')

def get_body_name(body):
    return get_body_info(body).body_name.decode(encoding='UTF-8')

def remove_body(body):
    if (CLIENT, body) in INFO_FROM_BODY:
        del INFO_FROM_BODY[CLIENT, body]
    return p.removeBody(body, physicsClientId=CLIENT)

def get_pose(body):
    return p.getBasePositionAndOrientation(body, physicsClientId=CLIENT)

def set_pose(body, pose):
    (point, quat) = pose
    p.resetBasePositionAndOrientation(body, point, quat, physicsClientId=CLIENT)

def get_point(body):
    return get_pose(body)[0]

def set_point(body, point):
    set_pose(body, (point, get_quat(body)))

def get_quat(body):
    return get_pose(body)[1] # [x,y,z,w]

def set_quat(body, quat):
    set_pose(body, (get_point(body), quat))

def get_euler(body):
    return euler_from_quat(get_quat(body))

def set_euler(body, euler):
    set_quat(body, quat_from_euler(euler))
# ---------------------

# def dump_body(body, fixed=False, links=True):
#     print('Body id: {} | Name: {} | Rigid: {} | Fixed: {}'.format(
#         body, get_body_name(body), is_rigid_body(body), is_fixed_base(body)))
#     for joint in get_joints(body):
#         if fixed or is_movable(body, joint):
#             dump_joint(body, joint)
#     if not links:
#         return
#     base_link = NULL_ID
#     num_visual = len(get_visual_data(body, base_link)) # TODO: try/catch
#     print('Link id: {} | Name: {} | Mass: {} | Collision: {} | Visual: {}'.format(
#         base_link, get_base_name(body), get_mass(body),
#         len(get_collision_data(body, base_link)), num_visual))
#     for link in get_links(body):
#         dump_link(body, link)
