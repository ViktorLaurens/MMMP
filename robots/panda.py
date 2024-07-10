import os 
from robots.robot import Robot
from utils.ik_utils import IK_info, get_ik_joints
from utils.pb_conf_utils import wait_for_duration
from utils.pb_joint_utils import set_joint_positions, get_movable_joints
from utils.pb_link_utils import link_from_name

class Panda(Robot):
    FRANKA_URDF = os.path.join(os.path.dirname(__file__), "../models/franka_panda/urdf/panda_arm_hand.urdf")
    PANDA_INFO = IK_info(base_link='panda_link0', ee_link='panda_link8', free_joints=['panda_joint7'])
    
    def __init__(self, fixed_base=True, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1), scale=1.0):
        super().__init__(Panda.FRANKA_URDF, fixed_base, base_position, base_orientation, scale)
        self.tool_link = link_from_name(self.robot_id, 'panda_hand')
        self.joints = get_movable_joints(self.robot_id)
        self.arm_joints = self.get_arm_joints()
        self.gripper_joints = self.get_gripper_joints()
        self.c_space = self.get_config_space(self.joints)
        self.arm_c_space = self.get_config_space(self.arm_joints)
        self.gripper_c_space = self.get_config_space(self.gripper_joints)
        self.dimension = len(self.joints)
        self.arm_dimension = len(self.arm_joints)
        self.gripper_dimension = len(self.gripper_joints)

        # Clear the terminal
        os.system('cls' if os.name == 'nt' else 'clear')

    # GETTERS
    def get_arm_joints(self):
        return get_movable_joints(self.robot_id)[:-2]

    def get_gripper_joints(self):
        return get_movable_joints(self.robot_id)[-2:]
    
    def get_arm_pose(self):
        return super().get_pose(self.arm_joints)
    
    def get_gripper_pose(self):
        return super().get_pose(self.gripper_joints)
    
    # SETTERS
    def set_arm_pose(self, pose):
        return super().set_pose(self.arm_joints, pose)
    
    def set_arm_in_standby(self): 
        standby_pose = (0, 0, 0, 0, 0, 0, 0)
        self.set_arm_pose(standby_pose)

    def set_arm_in_neutral(self):
        neutral_pose = (0, 0, 0, -1.51, 0, 1.877, 0)
        self.set_arm_pose(neutral_pose)
    
    def set_gripper_pose(self, pose):
        return super().set_pose(self.gripper_joints, pose)

    def set_gripper_open(self): 
        open_pose = (0.04, 0.04)
        self.set_gripper_pose(open_pose)

    def set_gripper_close(self):
        close_pose = (0, 0)
        self.set_gripper_pose(close_pose)

    def execute_arm_motion(self, arm_path):
        """
        Execute the planned arm motion on Franka Panda robot
        """
        for q in arm_path:
            self.set_arm_pose(q)
            wait_for_duration(0.15)

    def execute_gripper_motion(self, gripper_path):
        """
        Execute the planned gripper motion on Franka Panda robot
        """
        for q in gripper_path:
            self.set_gripper_pose(q)
            wait_for_duration(0.15)