import numpy as np
import pybullet as p
import pybullet_data
import time

from utils.traj_utils import calculate_lspb_trajectory, plot_acceleration_profile, plot_lspb_trajectory_for_joint, plot_velocity_profile

def joint_positions_from_trajectories(trajectories, t):
    joint_positions = []
    for joint in trajectories.keys(): 
        lspb_trajectory = trajectories[joint]
        for lspb in lspb_trajectory:
            if lspb.t1 <= t < lspb.t2:
                joint_positions.append(lspb.evaluate_position(t))
                break
    return np.array(joint_positions)

def main(): 
        # Example waypoints (in joint space)
    waypoints = np.array([
        [-1.5,  0.5, -2.5, -1.8, 0.0, 4.1, -0.1],
        [ 1.0,  1.5,  0.8, -1.5, 0.2, 3.2,  0.0],
        [ 0.5, -1.0, -1.6, -1.2, 0.5, 2.3,  0.1],
        [-2.0,  1.2,  0.5, -0.8, 0.7, 1.5,  0.3],
        [ 1.5, -1.7,  2.9, -0.5, 1.0, 0.7,  0.5]
    ])

    # Joint limits for the Panda robot
    joint_limits = {
        'q_max': [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159],
        'q_min': [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
    }
    
    # Velocity and acceleration limits
    velocity_limits = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])  # rad/s
    acceleration_limits = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0])  # rad/s²

    # LSPB trajectory
    t_final = 2.0
    effort_factor = 0.8
    trajectories = calculate_lspb_trajectory(waypoints, t_final, effort_factor, joint_limits, velocity_limits, acceleration_limits)
  
    # Connect to PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    
    # Load Franka Panda robot
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    # Simulation parameters
    time_step = 1.0 / 240.0
    p.setTimeStep(time_step)

    # Loop through trajectory points
    t_start = 0.0
    t_stop = trajectories[0][-1].t2
    for t in np.linspace(t_start, t_stop, num=int((t_stop - t_start) / time_step)):
        joint_positions = joint_positions_from_trajectories(trajectories, t)
        p.setJointMotorControlArray(robot_id, range(7), p.POSITION_CONTROL, joint_positions)
        p.stepSimulation()
        time.sleep(time_step)

    # Disconnect from PyBullet
    input("Press Enter to disconnect from PyBullet...")
    p.disconnect()
    return

if __name__ == '__main__':
    main()