import pybullet as p
import pybullet_data
import numpy as np
import time

from robots.panda import Panda
from utils.pb_conf_utils import add_data_path, connect, set_camera_pose

def initialize_pybullet():
    # Connect to PyBullet and set up the environment
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    # Load the plane and Panda URDF
    plane_id = p.loadURDF("plane.urdf")
    start_pos = [0, 0, 0]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF("franka_panda/panda.urdf", start_pos, start_orientation)

    return robot_id

def set_robot_pose(robot_id, pose):
    # Set the joint positions for the Franka Panda
    for i in range(len(pose)):
        p.resetJointState(robot_id, i, pose[i])

def main():
    connect(use_gui=True)
    add_data_path()
    set_camera_pose(camera_point=[0, -1.2, 1.2])
    ground = p.loadURDF("plane.urdf")

    panda = Panda(base_position=(0.5, 0, 0.01), base_orientation=(0, 0, 1, 0))  # Specify base position and orientation for robot1
    
    # A selected set of 'safe' poses
    safe_poses = [
        [0, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # central
        [np.pi/12, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # left
        [np.pi/6, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # left
        [np.pi/4, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # left
        [-np.pi/12, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # right
        [-np.pi/6, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # right
        [-np.pi/4, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # right
        [0, -np.pi/4, 0, -np.pi/4, 0, (-np.pi/4) - (-np.pi/4), np.pi/4],  # high
        [0, -np.pi/4, 0, -np.pi/2, 0, (-np.pi/4) - (-np.pi/2), np.pi/4],  # high
        [0, -np.pi/6, 0, -np.pi/2, 0, (-np.pi/6) - (-np.pi/2), np.pi/4],  # high
        [0, -np.pi/6, 0, -2*np.pi/3, 0, (-np.pi/6) - (-2*np.pi/3), np.pi/4],  # high
        [0, 0, 0, -np.pi/6, 0, 0 - (-np.pi/6), np.pi/4],  # high
        [0, 0, 0, -np.pi/12, 0, 0 - (-np.pi/12), np.pi/4],  # high
        [0, -np.pi/4, 0, -15*np.pi/18, 0, (-np.pi/4) - (-15*np.pi/18), np.pi/4],  # low
        [0, -np.pi/6, 0, -15*np.pi/18, 0, (-np.pi/6) - (-15*np.pi/18), np.pi/4],  # low
        [0, -np.pi/8, 0, -15*np.pi/18, 0, (-np.pi/8) - (-15*np.pi/18), np.pi/4],  # low
        [0, -np.pi/12, 0, -15*np.pi/18, 0, (-np.pi/12) - (-15*np.pi/18), np.pi/4],  # low
        [0, 0, 0, -15*np.pi/18, 0, 0 - (-15*np.pi/18), np.pi/4],  # low
        [np.pi/6, -np.pi/3, 0, -2*np.pi/3, 0, (-np.pi/3) - (-2*np.pi/3), np.pi/4],  # further left
        [np.pi/8, -np.pi/3, 0, -5*np.pi/6, 0, (-np.pi/3) - (-5*np.pi/6), np.pi/4],  # further left
        [-np.pi/6, -np.pi/3, 0, -2*np.pi/3, 0, (-np.pi/3) - (-2*np.pi/3), np.pi/4],  # further right
        [-np.pi/8, -np.pi/3, 0, -5*np.pi/6, 0, (-np.pi/3) - (-5*np.pi/6), np.pi/4],  # further right
        [0, -np.pi/3, 0, -np.pi/3, 0, (-np.pi/3) - (-np.pi/3), np.pi/4],  # high varied
        [0, -np.pi/4, np.pi/6, -15*np.pi/18, 0, (-np.pi/4) - (-15*np.pi/18), np.pi/4],  # low varied
        [0, -np.pi/6, np.pi/6, -15*np.pi/18, 0, (-np.pi/6) - (-15*np.pi/18), np.pi/4],  # low varied
        [np.pi/2, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # extended left
        [np.pi/3, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # extended left
        [-np.pi/2, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # extended right
        [-np.pi/3, -np.pi/4, 0, -3*np.pi/4, 0, (-np.pi/4) - (-3*np.pi/4), np.pi/4],  # extended right
        [0, -np.pi/4, 0, -5*np.pi/6, 0, (-np.pi/4) - (-5*np.pi/6), np.pi/4],  # extended low
        [0, -np.pi/4, 0, -2*np.pi/3, 0, (-np.pi/4) - (-2*np.pi/3), np.pi/4]  # extended low
    ]

    for i, pose in enumerate(safe_poses):
        panda.set_arm_pose(pose)
        print(f"Pose {i+1}: {pose}")
        input("Press Enter to continue to the next pose...")

if __name__ == "__main__":
    main()
