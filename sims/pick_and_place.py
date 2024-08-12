import math
import pybullet as p
import pybullet_data
import time

# Connect to physics server
p.connect(p.GUI)  # Use p.DIRECT for non-graphical version
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

# Load environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load plane, etc.
plane_id = p.loadURDF("plane.urdf")
# table_id = p.loadURDF("table/table.urdf", basePosition=[1.6, 0, 0])
cube_id = p.loadURDF("cube.urdf", basePosition=[0.6, 0.0, 0.0], globalScaling=0.05)

# Load Franka Panda robot
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Get joint and end effector information
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    print(p.getJointInfo(robot_id, i))

# Index of the end effector link
end_effector_index = 11

input("Press Enter to move the robot to the home position...")
# Home position of the robot
home_position = [0, -0.4, 0.0, -2.6, 0.0, 2.1, 0.7, 0.04, 0.04]
p.setJointMotorControlArray(robot_id, list(range(7)), p.POSITION_CONTROL, home_position[:7])
p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, 0.04)
p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, 0.04)

# Step the simulation to move the robot to the home position
for _ in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

input("Press Enter to pick and place the cube...")
# Define the pick position (above the cube)
pick_position = [0.62, 0.0, 0.1]  # Adjust the height above the cube
pick_orientation = p.getQuaternionFromEuler([0, math.pi, 0])

# Calculate inverse kinematics for the pick position
joint_angles = p.calculateInverseKinematics(robot_id, end_effector_index, pick_position, pick_orientation)

# Move the arm to the pick position
p.setJointMotorControlArray(robot_id, list(range(7)), p.POSITION_CONTROL, joint_angles[:7])

# Step the simulation to move the robot to the pick position
for _ in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

input("Press Enter to lower the end effector to the cube...")
# Lower the end effector to the cube
pick_position[2] = 0.02  # Lower the end effector to just above the cube
joint_angles = p.calculateInverseKinematics(robot_id, end_effector_index, pick_position, pick_orientation)
p.setJointMotorControlArray(robot_id, list(range(7)), p.POSITION_CONTROL, joint_angles[:7])

for _ in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

input("Press Enter to close the gripper and grasp the cube...")
# Close the gripper to grasp the cube
p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, 0.0, force=500)
p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, 0.0, force=500)

for _ in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

input("Press Enter to lift the cube...")
# Lift the cube
pick_position[2] = 0.2  # Lift the end effector with the cube
joint_angles = p.calculateInverseKinematics(robot_id, end_effector_index, pick_position, pick_orientation)
p.setJointMotorControlArray(robot_id, list(range(7)), p.POSITION_CONTROL, joint_angles[:7])

for _ in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

input("Press Enter to move the robot to the place position...")
# Move to place position
place_position = [0.4, 0.2, 0.2]  # Adjust to the desired place position
joint_angles = p.calculateInverseKinematics(robot_id, end_effector_index, place_position, pick_orientation)
p.setJointMotorControlArray(robot_id, list(range(7)), p.POSITION_CONTROL, joint_angles[:7])

for _ in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

input("Press Enter to lower the cube to the ground...")
# Lower the cube to the ground
place_position[2] = 0.02  # Lower the end effector to place the cube
joint_angles = p.calculateInverseKinematics(robot_id, end_effector_index, place_position, pick_orientation)
p.setJointMotorControlArray(robot_id, list(range(7)), p.POSITION_CONTROL, joint_angles[:7])

for _ in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

input("Press Enter to open the gripper and release the cube...")
# Open the gripper to release the cube
p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, 0.04, force=50)
p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, 0.04, force=50)

for _ in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

input("Press Enter to move back to the home position...")
# Move back to the home position
p.setJointMotorControlArray(robot_id, list(range(7)), p.POSITION_CONTROL, home_position[:7])

for _ in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

input("Press Enter to disconnect from the physics server...")
# Disconnect from the physics server
p.disconnect()
