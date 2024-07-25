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

# Disable the default velocity control
p.setJointMotorControlArray(robot, range(7), controlMode=p.VELOCITY_CONTROL, forces=[0]*7)

# Function to apply the trajectory
def apply_trajectory(robot, joint_trajectories, time_samples):
    # Ensure we match the simulation step with trajectory time resolution
    time_step = time_samples[1] - time_samples[0]
    p.setTimeStep(time_step)

    for i in range(len(time_samples)):
        positions = joint_trajectories[i]
        p.setJointMotorControlArray(robot, range(7), controlMode=p.POSITION_CONTROL, targetPositions=positions)
        p.stepSimulation()
        time.sleep(time_step)

# Example Trajectory Data (replace with your time-parameterized trajectory)
total_time = 10  # total time in seconds
num_samples = 1000  # number of samples in the trajectory
time_samples = np.linspace(0, total_time, num_samples)

# Example: Create a simple trajectory for each joint
joint_trajectories = []
for t in time_samples:
    positions = [np.sin(t), np.cos(t), np.sin(t/2), np.cos(t/2), np.sin(t/3), np.cos(t/3), np.sin(t/4)]
    joint_trajectories.append(positions)

# Apply the trajectory
apply_trajectory(robot, joint_trajectories, time_samples)

# Keep the simulation running for a bit after applying the trajectory
time.sleep(2)

# Disconnect from PyBullet
p.disconnect()
