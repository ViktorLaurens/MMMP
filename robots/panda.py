import os

import numpy as np 
import pybullet as p
from robots.robot import Robot
from utils.ik_utils import IK_info, calculate_ik, calculate_closest_ik
from utils.pb_conf_utils import wait_for_duration
from utils.pb_joint_utils import set_joint_positions, get_movable_joints
from utils.pb_link_utils import link_from_name
from scipy.spatial.transform import Rotation as R

class Panda(Robot):
    FRANKA_URDF = os.path.join(os.path.dirname(__file__), "../models/franka_panda/urdf/panda_arm_hand.urdf")
    PANDA_INFO = IK_info(base_link='panda_link0', ee_link='panda_link8', free_joints=['panda_joint7'])
    
    def __init__(self, fixed_base=True, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1), scale=1.0):
        super().__init__(Panda.FRANKA_URDF, fixed_base, base_position, base_orientation, scale)
        self.tool_link = link_from_name(self.r_id, 'panda_hand')
        print(f"Tool link: {self.tool_link}")
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
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def modified_homogeneous_transform(self, a, d, alpha, theta):
        """ Compute the modified Denavit-Hartenberg transformation matrix """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct, -st, 0, a],
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa, ca, ca*d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, q):
        """ Compute the forward kinematics of the panda robot"""
        # Initialize the transformation matrix with base position and orientation
        base_rot = R.from_quat(self.base_orientation).as_matrix()
        T_base = np.eye(4)
        T_base[:3, :3] = base_rot
        T_base[:3, 3] = self.base_position

        # Initialize the transformation matrix
        T = T_base

        # Compute the transformation matrix for each joint
        for i, params in enumerate(self.dh_params):
            if i == 7: 
                # Calculate the position and orientation of the actual end effector (called flange in franka panda documentation) 
                # which is only a translation with parameter d
                T_i = self.modified_homogeneous_transform(params["a"], params["d"], params["alpha"], params["theta"])
            elif i < 7:
                T_i = self.modified_homogeneous_transform(params["a"], params["d"], params["alpha"], q[i])
            T = np.matmul(T, T_i)
        return T
    
    def position_from_fk(self, q):
        """ Extract the position from the forward kinematics"""
        T = self.forward_kinematics(q)
        return T[:3, 3]
    
    def positions_from_fk(self, q):
        """ Extract the positions of the DH frames using forward kinematics"""
        #  Initialize the positions list
        positions = []

        # Initialize the transformation matrix with base position and orientation
        base_rot = R.from_quat(self.base_orientation).as_matrix()
        T_base = np.eye(4)
        T_base[:3, :3] = base_rot
        T_base[:3, 3] = self.base_position

        # Initialize the transformation matrix
        T = T_base

        #  Compute positions of frames using forward kinematics
        for i, params in enumerate(self.dh_params):
            if i == 7:
                T_i = self.modified_homogeneous_transform(params["a"], params["d"], params["alpha"], params["theta"])
            elif i < 7:
                T_i = self.modified_homogeneous_transform(params["a"], params["d"], params["alpha"], q[i])
            T = np.matmul(T, T_i)
            positions.append(T[:3, 3])
        return positions
    
    def orientation_from_fk(self, q):
        """ Extract the orientation from the forward kinematics"""
        T = self.forward_kinematics(q)
        return T[:3, :3]
    
    def rotation_matrix_to_quaternion(self, R):    
        """ Convert a rotation matrix to a quaternion """
        # Compute the eigenvalues and eigenvectors of rotation matrix
        eigvals, eigvecs = np.linalg.eig(R)
        # Extract the eigenvector corresponding to the eigenvalue of 1, this is the axis of rotation
        axis = eigvecs[:, np.isclose(eigvals, 1)]
        # Check if there is exactly one eigenvector with eigenvalue 1
        if axis.shape[1] != 1:
            raise ValueError("The rotation matrix does not have a unique axis of rotation")
        axis = axis[:, 0]  # Flatten the axis vector
        # The eigenvector is normalized to get the unit vector
        axis /= np.linalg.norm(axis)
        # Compute the angle of rotation from the trace of the rotation matrix (which is invariant under change of basis)
        angle = np.arccos((np.trace(R) - 1) / 2)
        # Compute the quaternion from the axis and angle of rotation
        q = np.concatenate([np.sin(angle / 2) * axis, [np.cos(angle / 2)]])
        return q
    
    def quaternion_from_fk(self, q):
        """ Extract the orientation from the forward kinematics"""
        R = self.orientation_from_fk(q)
        return self.rotation_matrix_to_quaternion(R)
    
    # INVERSE KINEMATICS
    def solve_arm_ik(self, targetPose): 
        return calculate_ik(self.r_id, self.tool_link, targetPose)[:-2]
    
    def solve_closest_arm_ik(self, targetPose, initialJointPositions):
        return calculate_closest_ik(self.r_id, self.tool_link, targetPose, initialJointPositions)[:-2]
    
    def solve_ik(self, target_position, target_orientation, default_joint_positions):
        joint_poses = p.calculateInverseKinematics(
        self.r_id, 
        endEffectorLinkIndex=8,  # End effector link index for Panda
        targetPosition=target_position, 
        targetOrientation=target_orientation,
        lowerLimits=[-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159],
        upperLimits=[2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159],
        jointRanges=[5.4874, 3.5674, 5.8014, 2.8903, 5.6130, 3.9724, 6.0318],
        restPoses=default_joint_positions,
        jointDamping=[1e-3, 1e-3, 1e+4, 1e-3, 1e+4, 1e+0, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6], 
        maxNumIterations=2000,
        residualThreshold=1e-4
        )
        return joint_poses[:7]

    # DISTANCE METRICS
    def distance_metric(self, q1, q2):
        """ Compute the specific distance metric for the panda robot."""
        return self.task_space_pose_distance(q1, q2)
    
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
    
    # task space distance metrics
    def task_space_pose_distance(self, q1, q2): 
        pos1 = self.positions_from_fk(q1)
        pos2 = self.positions_from_fk(q2)
        distance = 0
        for i in range(len(pos1)):
            distance += np.linalg.norm(pos1[i] - pos2[i])
        return distance
    
    def task_space_position_distance(self, q1, q2): 
        pos1 = np.array(self.position_from_fk(q1))
        pos2 = np.array(self.position_from_fk(q2))
        return np.linalg.norm(pos1 - pos2)

    def task_space_orientation_distance(self, q1, q2): 
        # Compute geodesic distance between two orientations
        option = 1
        if option == 1: 
            # Compute geodesic distance from orientation matrices
            R1 = np.array(self.orientation_from_fk(q1))
            R2 = np.array(self.orientation_from_fk(q2))
            R = np.dot(R1.T, R2)
            trace = np.trace(R)
            trace = min(3, max(-1, trace))
            return np.arccos((trace - 1) / 2)
        elif option == 2: 
            # Compute geodesic distance from quaternions
            R1 = np.array(self.orientation_from_fk(q1))
            R2 = np.array(self.orientation_from_fk(q2))
            q1 = self.rotation_matrix_to_quaternion(R1)
            q2 = self.rotation_matrix_to_quaternion(R2)
            return
        
    

    