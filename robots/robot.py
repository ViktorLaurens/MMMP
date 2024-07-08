import pybullet as p

from utils.pb_conf_utils import CLIENT, INFO_FROM_BODY, LockRenderer, ModelInfo, wait_for_duration
from utils.pb_joint_utils import get_joint_positions, get_movable_joints, set_joint_positions
from utils.planner_utils import get_custom_joint_intervals

class Robot:
    def __init__(self, urdf_path, fixed_base=False, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1), scale=1.0):
        self.urdf_path = urdf_path
        self.fixed_base = fixed_base
        self.base_position = base_position
        self.base_orientation = base_orientation
        self.scale = scale
        self.robot_id = self.load()
        self.joints = get_movable_joints(self.robot_id)
        self.config_space = self.get_config_space()
        self.dimension = len(self.joints)

    def load(self):
        with LockRenderer():
            if self.urdf_path.endswith('.urdf'):
                robot_id = p.loadURDF(self.urdf_path, basePosition=self.base_position, baseOrientation=self.base_orientation,
                             useFixedBase=self.fixed_base, globalScaling=self.scale, physicsClientId=CLIENT)
            else: 
                raise ValueError(self.urdf_path)
        INFO_FROM_BODY[robot_id] = ModelInfo(None, self.urdf_path, self.fixed_base, self.scale)
        return robot_id
    
    def get_config_space(self):
        return get_custom_joint_intervals(self.robot_id, self.joints)
   
    def set_pose(self, joints, pose):
        set_joint_positions(self.robot_id, joints, pose)

    def get_pose(self): 
        return get_joint_positions(self.robot_id, self.joints)
    
    def execute_motion(self, path):
        """
        Execute the planned motion on Franka Panda robot
        """
        for q in path:
            self.set_pose(self.joints, q)
            wait_for_duration(0.05)


