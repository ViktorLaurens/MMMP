import os

import numpy as np 
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
        self.tool_link = link_from_name(self.r_id, 'panda_hand')
        self.joints = get_movable_joints(self.r_id)
        self.arm_joints = self.get_arm_joints()
        self.gripper_joints = self.get_gripper_joints()
        self.c_space = self.get_config_space(self.joints)
        self.arm_c_space = self.get_config_space(self.arm_joints)
        self.gripper_c_space = self.get_config_space(self.gripper_joints)
        self.dimension = len(self.joints)
        self.arm_dimension = len(self.arm_joints)
        self.gripper_dimension = len(self.gripper_joints)

        self.dh_params = [
            {"a": 0, "d": 0.333, "alpha": 0, "theta": 0},
            {"a": 0, "d": 0, "alpha": -np.pi/2, "theta": 0},
            {"a": 0, "d": 0.316, "alpha": np.pi/2, "theta": 0},
            {"a": 0.0825, "d": 0, "alpha": np.pi/2, "theta": 0},
            {"a": -0.0825, "d": 0.384, "alpha": -np.pi/2, "theta": 0},
            {"a": 0, "d": 0, "alpha": np.pi/2, "theta": 0},
            {"a": 0.088, "d": 0, "alpha": np.pi/2, "theta": 0},
            {"a": 0, "d": 0.107, "alpha": 0, "theta": 0}
        ]

        # Clear the terminal
        os.system('cls' if os.name == 'nt' else 'clear')

    # GETTERS
    def get_arm_joints(self):
        return get_movable_joints(self.r_id)[:-2]

    def get_gripper_joints(self):
        return get_movable_joints(self.r_id)[-2:]
    
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
            wait_for_duration(0.05)

    def execute_gripper_motion(self, gripper_path):
        """
        Execute the planned gripper motion on Franka Panda robot
        """
        for q in gripper_path:
            self.set_gripper_pose(q)
            wait_for_duration(0.05)

    # FORWARD KINEMATICS
    def homogeneous_transform(self, a, d, alpha, theta):
        """ Compute the Denavit-Hartenberg transformation matrix """
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, q):
        """ Compute the forward kinematics of the panda robot"""
        # Initialize the transformation matrix
        T = np.eye(4)

        # Compute the transformation matrix for each joint
        for i, params in enumerate(self.dh_params):
            if i == 7: 
                # Calculate the position and orientation of the actual end effector (called flange in franka panda documentation) 
                # which is only a translation with parameter d
                T_i = self.homogeneous_transform(params["a"], params["d"], params["alpha"], params["theta"])
            elif i < 7:
                T_i = self.homogeneous_transform(params["a"], params["d"], params["alpha"], q[i])
            T = np.dot(T, T_i)
        return T
    
    def position_from_fk(self, q):
        """ Extract the position from the forward kinematics"""
        T = self.forward_kinematics(q)
        return T[:3, 3]
    
    def positions_from_fk(self, q):
        """ Extract the positions of the DH frames using forward kinematics"""
        #  Initialize the positions list
        positions = []

        # Initialize the transformation matrix
        T = np.eye(4)

        #  Compute positions of frames using forward kinematics
        for i, params in enumerate(self.dh_params):
            if i == 7:
                T_i = self.homogeneous_transform(params["a"], params["d"], params["alpha"], params["theta"])
            elif i < 7:
                T_i = self.homogeneous_transform(params["a"], params["d"], params["alpha"], q[i])
            T = np.dot(T, T_i)
            positions.append(T[:3, 3])
        return positions
    
    def orientation_from_fk(self, q):
        """ Extract the orientation from the forward kinematics"""
        T = self.forward_kinematics(q)
        return T[:3, :3]
    
    #  DISTANCE METRICS
    def distance_metric(self, q1, q2):
        """ Compute the specific distance metric for the panda robot."""
        return self.euclidean_distance_metric(q1, q2)
    
    def angular_distance_metric(self, q1, q2):
        """ Compute the angular distance metric for the panda robot using forward kinematics."""
        return np.linalg.norm(np.array(q1) - np.array(q2))
    
    def weighted_angular_distance_metric(self, q1, q2, weights):
        """ Compute the weighted angular distance metric for the panda robot using forward kinematics."""
        return np.linalg.norm(np.array(q1) - np.array(q2) * weights)
    
    def euclidean_distance_metric(self, q1, q2):
        """ Compute the euclidean distance metric for the panda robot using forward kinematics.
        This metric is chosen to be the difference in position of frames 3, 4, 5 and F 
        as consistent with the documentation on https://frankaemika.github.io/docs/control_parameters.html."""
        positions_q1 = self.positions_from_fk(q1)
        positions_q2 = self.positions_from_fk(q2) 
        return np.linalg.norm(np.array(positions_q1) - np.array(positions_q2))
    