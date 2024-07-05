import pybullet as p

from collections import namedtuple

from utils.pb_conf_utils import CLIENT
from utils.pb_data_utils import safe_zip

"""
JOINTS Utilities

This module provides utility functions and definitions for handling joints within simulations or visualizations.
"""
# ---------------------
# JOINTS variables
JOINT_TYPES = {
    p.JOINT_REVOLUTE: 'revolute', # 0
    p.JOINT_PRISMATIC: 'prismatic', # 1
    p.JOINT_SPHERICAL: 'spherical', # 2
    p.JOINT_PLANAR: 'planar', # 3
    p.JOINT_FIXED: 'fixed', # 4
    p.JOINT_POINT2POINT: 'point2point', # 5
    p.JOINT_GEAR: 'gear', # 6
}

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
                                     'qIndex', 'uIndex', 'flags',
                                     'jointDamping', 'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                                     'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
                                     'parentFramePos', 'parentFrameOrn', 'parentIndex'])

JointState = namedtuple('JointState', ['jointPosition', 'jointVelocity',
                                       'jointReactionForces', 'appliedJointMotorTorque'])
# ---------------------

# ---------------------
# Getters and setters
def get_num_joints(body):
    return p.getNumJoints(body, physicsClientId=CLIENT)

def get_joints(body):
    return list(range(get_num_joints(body)))

def get_joint_info(body, joint):
    return JointInfo(*p.getJointInfo(body, joint, physicsClientId=CLIENT))

def get_joint_name(body, joint):
    return get_joint_info(body, joint).jointName.decode('UTF-8')

def get_joint_names(body, joints):
    return [get_joint_name(body, joint) for joint in joints]

def joint_from_name(body, name):
    for joint in get_joints(body):
        if get_joint_name(body, joint) == name:
            return joint
    raise ValueError(body, name)

def joints_from_names(body, names):
    return tuple(joint_from_name(body, name) for name in names)

def get_joint_type(body, joint):
    return get_joint_info(body, joint).jointType

def is_fixed(body, joint):
    return get_joint_type(body, joint) == p.JOINT_FIXED

def is_movable(body, joint):
    return not is_fixed(body, joint)

def prune_fixed_joints(body, joints):
    return [joint for joint in joints if is_movable(body, joint)]

def get_movable_joints(body):
    return prune_fixed_joints(body, get_joints(body))

def set_joint_position(body, joint, value):
    # TODO: remove targetVelocity=0
    p.resetJointState(body, joint, targetValue=value, targetVelocity=0, physicsClientId=CLIENT)

def get_joint_state(body, joint):
    return JointState(*p.getJointState(body, joint, physicsClientId=CLIENT))

def get_joint_position(body, joint):
    return get_joint_state(body, joint).jointPosition

def set_joint_positions(body, joints, values):
    for joint, value in safe_zip(joints, values):
        set_joint_position(body, joint, value)

def get_joint_positions(body, joints): # joints=None):
    return tuple(get_joint_position(body, joint) for joint in joints)
# ---------------------