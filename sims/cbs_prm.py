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
from planners.prm.pybullet.decoupled.cbsprm import CBSPRM
from robots.panda import Panda
from utils.pb_conf_utils import add_data_path, connect, disconnect, pause_sim, set_camera_pose

def main():
    connect(use_gui=True)
    add_data_path()
    set_camera_pose(camera_point=[0, -1.2, 1.2])
    ground = p.loadURDF("plane.urdf")

    panda1 = Panda(base_position=(0.9, 0, 0.01), base_orientation=(0, 0, 1, 0))  # Specify base position and orientation for robot1
    panda2 = Panda(base_position=(-0.9, 0, 0.01), base_orientation=(0, 0, 0, 1))  # Specify base position and orientation for robot2

    # Disable collision between the robot and the ground plane
    # p.setCollisionFilterPair(panda1.r_id, ground, -1, -1, enableCollision=0)
    # p.setCollisionFilterPair(panda2.r_id, ground, -1, -1, enableCollision=0)

    reference_joint_positions = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
    # reference_joint_positions = [0, 0, 0, 0, 0, 0, 0]
    panda1.set_arm_pose(reference_joint_positions)
    panda2.set_arm_pose(reference_joint_positions)

    # pos1 = panda1.position_from_fk(reference_joint_positions)
    # pos2 = panda2.position_from_fk(reference_joint_positions)
    # print(f"Position of panda1: {pos1}")
    # print(f"Position of panda2: {pos2}")
    # positions1 = panda1.positions_from_fk(reference_joint_positions)
    # positions2 = panda2.positions_from_fk(reference_joint_positions)
    # print(f"\nPositions of panda1: {positions1}")
    # print(f"\nPositions of panda2: {positions2}")
    # input()

    # Calculate and set start pose of arm 1
    tool_position_start1 = (-0.4, -0.4, 0.5)
    tool_orientation_start1 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    arm1_start = panda1.solve_ik(tool_position_start1, tool_orientation_start1, reference_joint_positions)
    arm1_start = tuple(round(c, 3) for c in arm1_start)

    if arm1_start is not None:
        pause_sim('Show start pose for first panda... (Press Enter to continue)')
        # print("IK Configuration for panda 1 start pose:", arm1_start)
        panda1.set_arm_pose(arm1_start)
        print(f"Start q for panda 1: {arm1_start}")
        print(f"Start position for panda 1: {panda1.position_from_fk(arm1_start)}")
    else:
        print("No IK solution found for start pose of second panda.")


    # Calculate and set goal pose of arm 1
    tool_position_goal1 = (-0.4, 0.4, 0.5)
    tool_orientation_goal1 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    arm1_goal = panda1.solve_ik(tool_position_goal1, tool_orientation_goal1, arm1_start)
    arm1_goal = tuple(round(c, 3) for c in arm1_goal)
    
    if arm1_goal is not None:
        pause_sim('Show goal pose for first Panda... (Press Enter to continue)')
        panda1.set_arm_pose(arm1_goal)
        print(f"Goal q for panda 1: {arm1_goal}")
        print(f"Goal position for panda 1: {panda1.position_from_fk(arm1_goal)}")
    else:
        print("No IK solution found for goal pose of first panda.")


    # Calculate and set start of arm 2
    tool_position_start2 = (0.4, 0.4, 0.5)
    tool_orientation_start2 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    arm2_start = panda2.solve_ik(tool_position_start2, tool_orientation_start2, reference_joint_positions)
    arm2_start = tuple(round(c, 3) for c in arm2_start)
    
    if arm2_start is not None:
        pause_sim('Show start pose for second panda... (Press Enter to continue)')
        panda2.set_arm_pose(arm2_start)
        print(f"Start q for panda 2: {arm2_start}")
        print(f"Start position for panda 2: {panda2.position_from_fk(arm2_start)}")
    else:
        print("No IK solution found for start pose of second panda.")


    # Calculate and set goal of arm 2
    tool_position_goal2 = (0.4, -0.4, 0.5)
    tool_orientation_goal2 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    arm2_goal = panda2.solve_ik(tool_position_goal2, tool_orientation_goal2, arm2_start)
    arm2_goal = tuple(round(c, 3) for c in arm2_goal)
    
    if arm2_goal is not None:
        pause_sim('Show goal pose for second panda... (Press Enter to continue)')
        panda2.set_arm_pose(arm2_goal)
        print(f"Goal q for panda 2: {arm2_goal}")
        print(f"Goal position for panda 2: {panda2.position_from_fk(arm2_goal)}")
    else:
        print("No IK solution found for goal pose of second panda.")


    agents = [
            {"name": "agent1", "start": arm1_start, "goal": arm1_goal, "model": panda1}, 
            {"name": "agent2", "start": arm2_start, "goal": arm2_goal, "model": panda2}
        ]
    
    obstacles = [ground]

    pause_sim('Load environment and reset poses to start... (Press Enter to continue)')
    env = Environment(agents, obstacles)

    pause_sim('Learn?')
    start_time = time.time()
    directory = os.path.dirname(__file__)  # Get the directory of the current file
    relative_path = '../res/images/task_space_roadmap2_adjusted_pruned.csv'
    filename = os.path.join(directory, relative_path)
    prm = CBSPRM(env, load_roadmap=filename, maxdist=0.1, k1=11, k2=10, build_type='n', prm_type='distance', n=20, t=10, time_step=0.01, local_step=0.02)
    learn_duration = time.time()-start_time
    print(f"Learning duration: {learn_duration}")
    # print(f"Edges: {prm.edge_dicts}")
    print(f'Average degree: {np.mean(list(len(prm.edge_dicts[0][n_id]) for n_id in prm.edge_dicts[0].keys()))}')
    panda1.set_arm_pose(arm1_start)
    panda2.set_arm_pose(arm2_start)

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

    frames = None
    # pause_sim('execute joint motion?')
    # frames = env.execute_joint_motion_capturing_frames(paths)
    # frames = env.execute_joint_motion(paths)
    pause_sim('execute interpolated motion?')
    env.execute_interpolated_motion(prm, paths, t_final=2.0)
    pause_sim('execute smooth motion?')
    env.execute_smooth_motion(prm, paths, t_final=2.0, effort_factor=0.8)
    pause_sim('Disconnect?')
    disconnect()

    if frames is not None: 
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