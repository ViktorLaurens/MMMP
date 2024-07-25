import pybullet as p
import pybullet_data
import numpy as np
import time

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load Plane and Panda Robot URDF
p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Define the end-effector link index (Panda's end effector is link 11)
end_effector_index = 11

# Define the initial joint positions (close to current pose)
initial_joint_positions = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]

# Reset the robot to the initial position
for i in range(len(initial_joint_positions)):
    p.resetJointState(robot, i, initial_joint_positions[i])

p.stepSimulation()  
time.sleep(5)

# Define the target position and orientation for the end-effector
target_position = [0.5, 0, 0.5]  # Example target position (x, y, z)
target_orientation = p.getQuaternionFromEuler([0, -np.pi/2, 0])  # Example target orientation (Euler angles)

# Compute the inverse kinematics for the target pose
ik_solution = p.calculateInverseKinematics(
    robot,
    end_effector_index,
    target_position,
    targetOrientation=target_orientation,
    restPoses=initial_joint_positions  # Starting from initial positions to find the closest solution
)

# Apply the IK solution to the robot
for i in range(len(initial_joint_positions)):
    p.resetJointState(robot, i, ik_solution[i])

# Step simulation to visualize the result
p.stepSimulation()

# Keep the simulation running for a bit to visualize
time.sleep(5)

# Disconnect from PyBullet
p.disconnect()
