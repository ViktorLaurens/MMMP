import os 
from robots.robot import Robot
from utils.ik_utils import IK_info, get_ik_joints
from utils.pb_conf_utils import wait_for_duration
from utils.pb_joint_utils import set_joint_positions
from utils.pb_link_utils import link_from_name

class Panda(Robot):
    FRANKA_URDF = os.path.join(os.path.dirname(__file__), "../models/franka_description/robots/panda_arm_hand.urdf")
    PANDA_INFO = IK_info(base_link='panda_link0', ee_link='panda_link8', free_joints=['panda_joint7'])
    
    def __init__(self, fixed_base=False, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1), scale=1.0):
        super().__init__(Panda.FRANKA_URDF, fixed_base, base_position, base_orientation, scale)
        self.standby_pose = (0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.neutral_pose = (0, 0, 0, -1.51, 0, 1.877, 0, 0.04, 0.04)
        self.tool_link = link_from_name(self.robot_id, 'panda_hand')
        self.joints = get_ik_joints(self.robot_id, Panda.PANDA_INFO, self.tool_link)
        # self.gripper_joints = get_ik_joints(self.robot_id, Panda.PANDA_INFO, self.tool_link)
        self.config_space = self.get_config_space()
        self.dimension = len(self.joints)

    def set_pose(self, pose):
        set_joint_positions(self.robot_id, self.joints, pose)

    def set_in_standby(self): 
        self.set_pose(self.standby_pose)

    def set_in_neutral(self):
        self.set_pose(self.neutral_pose)
    
    def execute_motion(self, arm_path):
        """
        Execute the planned motion on Franka Panda robot
        """
        for q in arm_path:
            set_joint_positions(self.robot_id, self.joints, q)
            wait_for_duration(0.15)