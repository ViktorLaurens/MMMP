import time
import imageio
import numpy as np
import pybullet as p

# Define the path to your project root directory
project_root = 'C:\\Users\\vikto\\MMMP'

# Add the project root directory to sys.path if it's not already included
import sys
import os
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from planners.prm.pybullet.env import Environment
from planners.prm.pybullet.coupled.degree_coupled_prm import KDTreeKNNCoupledPRM 
from robots.panda import Panda
from utils.ik_utils import calculate_arm_ik
from utils.pb_conf_utils import add_data_path, connect, disconnect, pause_sim, set_camera_pose

 # parameter
# N_SAMPLES = 100  # number of samples
# N_KNN = 5  # number of edge from one sampled point
# MAX_EDGE_LEN = 2  # [m] Maximum edge length
# TYPE = 'nn' # 'distance' or 'nn'
# LOCAL_STEP = 0.05  # [rad] Step size for local planner

# constants
# INF = float("inf")

def main():
    connect(use_gui=True)
    add_data_path()
    set_camera_pose(camera_point=[0, -1.2, 1.2])
    ground = p.loadURDF("plane.urdf")

    panda1 = Panda(base_position=(0.6, 0, 0.02), base_orientation=(0, 0, 1, 0))  # Specify base position and orientation for robot1
    panda2 = Panda(base_position=(-0.6, 0, 0.02), base_orientation=(0, 0, 0, 1))  # Specify base position and orientation for robot2

    # Disable collision between the robot and the ground plane
    p.setCollisionFilterPair(panda1.r_id, ground, -1, -1, enableCollision=0)
    p.setCollisionFilterPair(panda2.r_id, ground, -1, -1, enableCollision=0)

    # Calculate and set goal pose of arm 1
    tool_position_goal1 = (-0.3, 0.4, 0.5)
    tool_orientation_goal1 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    tool_pose_goal1 = (tool_position_goal1, tool_orientation_goal1)
    arm1_goal = calculate_arm_ik(panda1.r_id, panda1.tool_link, tool_pose_goal1)
    arm1_goal = tuple(round(c, 3) for c in arm1_goal)
    
    arm1_goal = (-0.319, 1.221, -0.255, -0.199, 0.244, 1.456, -2.724)
    print(arm1_goal)

    if arm1_goal is not None:
        pause_sim('Show goal pose for first Panda?')
        # print("IK Configuration for panda 1 goal pose:", arm1_goal)
        panda1.set_arm_pose(arm1_goal)
    else:
        print("No IK solution found for goal pose of first panda.")

    # Calculate and set start pose of arm 1
    tool_position_start1 = (-0.3, -0.4, 0.5)
    tool_orientation_start1 = p.getQuaternionFromEuler([np.radians(-180), 0, 0])
    tool_pose_start1 = (tool_position_start1, tool_orientation_start1)
    arm1_start = calculate_arm_ik(panda1.r_id, panda1.tool_link, tool_pose_start1)
    arm1_start = tuple(round(c, 3) for c in arm1_start)
    
    arm1_start = (0.41, 1.103, -0.243, -0.419, 0.219, 1.558, -2.045)
    print(arm1_start)

    if arm1_start is not None:
        pause_sim('Show start pose for first panda?')
        # print("IK Configuration for panda 1 start pose:", arm1_start)
        panda1.set_arm_pose(arm1_start)
    else:
        print("No IK solution found for start pose of second panda.")

    # Calculate and set goal of arm 2
    tool_position_goal2 = (0.1, -0.4, 0.5)
    tool_orientation_goal2 = p.getQuaternionFromEuler([np.radians(-180), 0, 0])
    tool_pose_goal2 = (tool_position_goal2, tool_orientation_goal2)
    arm2_goal = calculate_arm_ik(panda2.r_id, panda2.tool_link, tool_pose_goal2)
    arm2_goal = tuple(round(c, 3) for c in arm2_goal)
    
    arm2_goal = (-0.416, 1.116, -0.012, -0.293, 0.011, 1.434, 0.366)
    print(arm2_goal)

    if arm2_goal is not None:
        pause_sim('Show goal pose for second panda?')
        # print("IK Configuration for panda 2 goal pose:", arm2_goal)
        panda2.set_arm_pose(arm2_goal)
    else:
        print("No IK solution found for goal pose of second panda.")

    # Calculate and set start of arm 2
    tool_position_start2 = (0.1, 0.4, 0.5)
    tool_orientation_start2 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    tool_pose_start2 = (tool_position_start2, tool_orientation_start2)
    arm2_start = calculate_arm_ik(panda2.r_id, panda2.tool_link, tool_pose_start2)
    arm2_start = tuple(round(c, 3) for c in arm2_start)
    
    arm2_start = (0.413, 1.057, 0.02, -0.4, -0.018, 1.484, 1.207)
    print(arm2_start)

    if arm2_start is not None:
        pause_sim('Show start pose for second panda?')
        # print("IK Configuration for panda 2 start pose:", arm2_start)
        panda2.set_arm_pose(arm2_start)
    else:
        print("No IK solution found for start pose of second panda.")

    agents = [
            {"name": "agent1", "start": arm1_start, "goal": arm1_goal, "model": panda1, "roadmap": None}, 
            {"name": "agent2", "start": arm2_start, "goal": arm2_goal, "model": panda2, "roadmap": None}
        ]
    
    obstacles = [ground]

    env = Environment(agents, obstacles)

    pause_sim('Learn?')
    start_time = time.time()
    prm = KDTreeKNNCoupledPRM(env, max_edge_len=5, n_knn=5)
    # prm = KDTreeDistanceCoupledPRM(env, max_edge_len=1)
    learn_duration = time.time()-start_time
    print(f"Learning duration: {learn_duration}")
    # print(f"Edges: {prm.edge_dicts}")

    panda1.set_arm_pose(arm1_start)
    panda2.set_arm_pose(arm2_start)

    # # Plot the roadmap
    # pause_sim('Plot roadmap?')
    # prm.plot_roadmap_2D_subspace()

    # change roadmap parameter max_edge_len or n_knn
    while True: 
        avg_degree = prm.compute_combined_avg_degree()
        print(f"\nCurrent parameters: \nn_nodes = {len(prm.nodes)}\nmax_edge_len = {prm.max_edge_len}\navg_degree = {avg_degree}")
        print("\nOptions: \n1 - Add samples\n2 - Change max_edge_len\n3 - Exit")
        choice = input("Enter number: ")
        if choice=='1':
            n_to_add = int(input("Enter the number of samples to add: "))
            prm.add_and_update(n_to_add)
            print(f"Added {n_to_add} samples.")
        elif choice=='2': 
            new_len = float(input("Enter new max_edge_len: "))
            prm.max_edge_len = new_len
            prm.update_roadmap()
            print(f"Updated max_edge_len to {new_len}.")
        elif choice == '3':
            print("Exiting parameter update...")
            break
        else:
            print("Invalid option, please enter an integer between 1 and 3.")
            continue
        # prm.plot_roadmap_2D_subspace()

    # Plan motion
    pause_sim('Query?')
    start_time = time.time()
    paths = prm.query()
    query_duration = time.time()-start_time
    if not paths:
        print("Solution not found")
        pause_sim('Disconnect?')
        disconnect()
        return
    print(f"Solution: {paths}")
    print(f"Query duration: {query_duration}")

    pause_sim('execute joint motion?')
    frames = env.execute_joint_motion_capturing_frames(paths)
    pause_sim('Disconnect?')
    disconnect()

    flag = input('Save video? (y/N): ')
    if flag.lower() == 'y':
        # Define where to save the video
        now = time.strftime("%Y%m%d_%H%M%S")
        directory = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Videos'
        filename = f'\simulation_{len(agents)}pandas_{panda1.base_position}_{now}.mp4'

        # Save the captured frames as a video
        imageio.mimsave(directory + filename, frames, fps=30)  # Define the FPS as needed
        print(f"Video saved as {directory + filename}")
    else: 
        print("Video not saved.")

if __name__ == "__main__":
    main()