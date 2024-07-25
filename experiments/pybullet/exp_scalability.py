import time
from matplotlib import pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Circle
import pandas as pd
import seaborn as sns

# Define the path to your project root directory
project_root = 'C:\\Users\\vikto\\MMMP'

# Add the project root directory to sys.path if it's not already included
import sys
import os
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import pybullet as p
from utils.pb_conf_utils import add_data_path, connect, disconnect, pause_sim, set_camera_pose
from planners.prm.pybullet.env import Environment
from robots.panda import Panda
from planners.prm.pybullet.decoupled.prioritized_prm import PrioritizedPRM
from planners.prm.pybullet.decoupled.cbsprm import CBSPRM


def load_scenario_1(): 
    # Load Panda's in pybullet
    base_pos1 = (0.55, 0, 0.01)
    base_ori1 = (0, 0, 1, 0)
    panda1 = Panda(base_position=base_pos1, base_orientation=base_ori1)

    base_pos2 = (-0.55, 0, 0.01)
    base_ori2 = (0, 0, 0, 1)
    panda2 = Panda(base_position=base_pos2, base_orientation=base_ori2)

    # reference
    reference_joint_positions = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]

    # Determine goal and start configurations for pandas from task space poses of ee using IK
    # First panda goal and start poses
    # Start pose
    ee_pos_s1 = (-0., -0., 0.3)
    ee_ori_s1 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    ee_pose_s1 = (ee_pos_s1, ee_ori_s1)
    s1 = panda1.solve_closest_arm_ik(ee_pose_s1, reference_joint_positions)
    s1 = (0., 0., 0., -3*np.pi/4, 0., 3*np.pi/4, np.pi/4)
    s1 = tuple(round(c, 3) for c in s1)
    print('s1', s1)

    if s1 is not None:
        panda1.set_arm_pose(s1)
        pause_sim('Start pose for first Panda. Enter to continue.')
    else:
        print("No IK solution found for start pose of first panda.")

    # Goal pose
    ee_pos_g1 = (-0.2, 0., 0.7)
    ee_ori_g1 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    ee_pose_g1 = (ee_pos_g1, ee_ori_g1)
    g1 = panda1.solve_closest_arm_ik(ee_pose_g1, reference_joint_positions)
    g1 = (0., 0., 0., -np.pi/2 + np.pi/16, 0., np.pi/2 - np.pi/16, np.pi/4)
    g1 = tuple(round(c, 3) for c in g1)
    print('g1', g1)
    # g1 = (-0.319, 1.221, -0.255, -0.199, 0.244, 1.456, -2.724)

    if g1 is not None:
        panda1.set_arm_pose(g1)
        pause_sim('Goal pose for first Panda. Enter to continue.')
    else:
        print("No IK solution found for goal pose of first panda.")

    # Second panda goal and start poses
    # Start pose
    ee_pos_s2 = (0., 0., 0.7)
    ee_ori_s2 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    ee_pose_s2 = (ee_pos_s2, ee_ori_s2)
    s2 = panda2.solve_closest_arm_ik(ee_pose_s2, reference_joint_positions)
    s2 = (0., 0., 0., -np.pi/2 + np.pi/16, 0., np.pi/2 - np.pi/16, np.pi/4)
    s2 = tuple(round(c, 3) for c in s2)
    print('s2', s2)

    if s2 is not None:
        panda2.set_arm_pose(s2)
        pause_sim('Start pose for second Panda. Enter to continue.')
    else:
        print("No IK solution found for start pose of second panda.")
    
    # Goal pose
    ee_pos_g2 = (0., -0., 0.3)
    ee_ori_g2 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    ee_pose_g2 = (ee_pos_g2, ee_ori_g2)
    g2 = panda2.solve_closest_arm_ik(ee_pose_g2, reference_joint_positions)
    g2 = (0., 0., 0., -3*np.pi/4, 0., 3*np.pi/4, np.pi/4)
    g2 = tuple(round(c, 3) for c in g2)
    print('g2', g2)

    if g2 is not None:
        panda2.set_arm_pose(g2)
        pause_sim('Goal pose for second Panda. Enter to continue.')
    else:
        print("No IK solution found for goal pose of second panda.")

    # Define agents
    agent1 = {"name": "agent1", "start": s1, "goal": g1, "model": panda1}
    agent2 = {"name": "agent2", "start": s2, "goal": g2, "model": panda2}
    agents = [agent1, agent2]

    return agents

def load_scenario_2(): 
    # Load Panda's in pybullet
    base_pos1 = (0.6, 0, 0.01)
    base_ori1 = (0, 0, 1, 0)
    panda1 = Panda(base_position=base_pos1, base_orientation=base_ori1)

    base_pos2 = (-0.6, 0, 0.01)
    base_ori2 = (0, 0, 0, 1)
    panda2 = Panda(base_position=base_pos2, base_orientation=base_ori2)

    # reference
    reference_joint_positions = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]

    # Determine goal and start configurations for pandas from task space poses of ee using IK
    # First panda goal and start poses
    # Start pose
    ee_pos_s1 = (-0., -0., 0.3)
    ee_ori_s1 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    ee_pose_s1 = (ee_pos_s1, ee_ori_s1)
    s1 = panda1.solve_closest_arm_ik(ee_pose_s1, reference_joint_positions)
    s1 = (0., 0., 0., -3*np.pi/4, 0., 3*np.pi/4, np.pi/4)
    s1 = tuple(round(c, 3) for c in s1)
    print('s1', s1)

    if s1 is not None:
        panda1.set_arm_pose(s1)
        pause_sim('Start pose for first Panda. Enter to continue.')
    else:
        print("No IK solution found for start pose of first panda.")

    # Goal pose
    ee_pos_g1 = (-0.2, 0., 0.7)
    ee_ori_g1 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    ee_pose_g1 = (ee_pos_g1, ee_ori_g1)
    g1 = panda1.solve_closest_arm_ik(ee_pose_g1, reference_joint_positions)
    g1 = (0., 0., 0., -np.pi/2 + np.pi/16, 0., np.pi/2 - np.pi/16, np.pi/4)
    g1 = tuple(round(c, 3) for c in g1)
    print('g1', g1)
    # g1 = (-0.319, 1.221, -0.255, -0.199, 0.244, 1.456, -2.724)

    if g1 is not None:
        panda1.set_arm_pose(g1)
        pause_sim('Goal pose for first Panda. Enter to continue.')
    else:
        print("No IK solution found for goal pose of first panda.")

    # Second panda goal and start poses
    # Start pose
    ee_pos_s2 = (0., 0., 0.7)
    ee_ori_s2 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    ee_pose_s2 = (ee_pos_s2, ee_ori_s2)
    s2 = panda2.solve_closest_arm_ik(ee_pose_s2, reference_joint_positions)
    s2 = (0., 0., 0., -3*np.pi/4, 0., 3*np.pi/4, np.pi/4)
    s2 = tuple(round(c, 3) for c in s2)
    print('s2', s2)

    if s2 is not None:
        panda2.set_arm_pose(s2)
        pause_sim('Start pose for second Panda. Enter to continue.')
    else:
        print("No IK solution found for start pose of second panda.")
    
    # Goal pose
    ee_pos_g2 = (0., -0., 0.3)
    ee_ori_g2 = p.getQuaternionFromEuler([np.radians(180), 0, 0])
    ee_pose_g2 = (ee_pos_g2, ee_ori_g2)
    g2 = panda2.solve_closest_arm_ik(ee_pose_g2, reference_joint_positions)
    g2 = (0., 0., 0., -np.pi/2 + np.pi/16, 0., np.pi/2 - np.pi/16, np.pi/4)
    g2 = tuple(round(c, 3) for c in g2)
    print('g2', g2)

    if g2 is not None:
        panda2.set_arm_pose(g2)
        pause_sim('Goal pose for second Panda. Enter to continue.')
    else:
        print("No IK solution found for goal pose of second panda.")

    # Define agents
    agent1 = {"name": "agent1", "start": s1, "goal": g1, "model": panda1}
    agent2 = {"name": "agent2", "start": s2, "goal": g2, "model": panda2}
    agents = [agent1, agent2]

    return agents

def calculate_base_positions(num_arms, radius=0.7):
    positions = []
    orientations = []
    angle_increment = 2 * np.pi / num_arms
    for i in range(num_arms):
        angle = i * angle_increment
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        positions.append((x, y, 0.01))
        # Orientations facing the center
        orientation = p.getQuaternionFromEuler([0, 0, angle + np.pi])
        orientations.append(orientation)
    return positions, orientations

def load_scenario_n(num_arms):
    # Calculate base positions and orientations
    base_positions, base_orientations = calculate_base_positions(num_arms)

    pandas = []
    for base_pos, base_ori in zip(base_positions, base_orientations):
        pandas.append(Panda(base_position=base_pos, base_orientation=base_ori))

    start_poses = []
    goal_poses = []
    
    for i, panda in enumerate(pandas):
        start_pose = (0., 0., 0., -3*np.pi/4, 0., 3*np.pi/4, np.pi/4)
        start_pose = tuple(round(c, 3) for c in start_pose)
        start_poses.append(start_pose)
        
        goal_pose = (0., 0., 0., -np.pi/2 + np.pi/16, 0., np.pi/2 - np.pi/16, np.pi/4)
        goal_pose = tuple(round(c, 3) for c in goal_pose)
        goal_poses.append(goal_pose)

    agents = []
    for i, panda in enumerate(pandas):
        agent = {
            "name": f"agent{i+1}",
            "start": start_poses[i],
            "goal": goal_poses[i],
            "model": panda
        }
        agents.append(agent)

    return agents

# def load_scenario_2():
#     s1 = np.array([0, 0])
#     g1 = np.array([np.pi/3, 0])
#     model1 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5.6, -4]))
#     model1.set_pose(s1)

#     s2 = np.array([np.pi/3*2, 0])
#     g2 = np.array([np.pi, 0])
#     model2 = PlanarArm(np.array([3, 3]), base_pos=np.array([5.6, -4]))
#     model2.set_pose(s2)

#     s3 = np.array([-np.pi/3*2, 0])
#     g3 = np.array([-np.pi/3, 0])
#     model3 = PlanarArm(np.array([3, 3]), base_pos=np.array([0, 4.6]))
#     model3.set_pose(s3)
    
#     agents = [
#             {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None}, 
#             {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}, 
#             {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None}        
#             ]
    
#     return agents

# def load_scenario_3():
#     s1 = np.array([0,0])
#     g1 = np.array([np.pi/2,0])
#     model1 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5, -5]))
#     model1.set_pose(s1)

#     s2 = np.array([-np.pi/2, 0])
#     g2 = np.array([0, 0])
#     model2 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5, 5]))
#     model2.set_pose(s2)

#     s3 = np.array([-np.pi, 0])
#     g3 = np.array([-np.pi/2, 0])
#     model3 = PlanarArm(np.array([3, 3]), base_pos=np.array([5, 5]))
#     model3.set_pose(s3)

#     s4 = np.array([np.pi/2, 0])
#     g4 = np.array([np.pi, 0])
#     model4 = PlanarArm(np.array([3, 3]), base_pos=np.array([5, -5]))
#     model4.set_pose(s4)
    
#     agents = [
#             {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None}, 
#             {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}, 
#             {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None},  
#             {"name": "agent4", "start": s4, "goal": g4, "model": model4, "roadmap": None}              
#             ]
    
#     return agents

# def load_scenario_4():
#     s1 = np.array([np.pi/5*3, 0])
#     g1 = np.array([0, 0])
#     model1 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5, -7]))
#     model1.set_pose(s1)

#     s2 = np.array([np.pi/5, 0])
#     g2 = np.array([-np.pi/5*2, 0])
#     model2 = PlanarArm(np.array([3, 3]), base_pos=np.array([-7.5, 1.5]))
#     model2.set_pose(s2)

#     s3 = np.array([-np.pi/5, 0])
#     g3 = np.array([-np.pi/5*4, 0])
#     model3 = PlanarArm(np.array([3, 3]), base_pos=np.array([0, 7]))
#     model3.set_pose(s3)

#     s4 = np.array([-np.pi/5*3, 0])
#     g4 = np.array([-np.pi, 0])
#     model4 = PlanarArm(np.array([3, 3]), base_pos=np.array([7.5, 1.5]))
#     model4.set_pose(s4)

#     s5 = np.array([np.pi, 0])
#     g5 = np.array([np.pi/5*2, 0])
#     model5 = PlanarArm(np.array([3, 3]), base_pos=np.array([5, -7]))
#     model5.set_pose(s5)
    
#     agents = [
#             {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None}, 
#             {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}, 
#             {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None},  
#             {"name": "agent4", "start": s4, "goal": g4, "model": model4, "roadmap": None}, 
#             {"name": "agent5", "start": s5, "goal": g5, "model": model5, "roadmap": None}             
#             ]
    
#     return agents

# def load_scenario_5():
#     s1 = np.array([0, 0])
#     g1 = np.array([np.pi/4, 0])
#     model1 = PlanarArm(np.array([2, 2]), base_pos=np.array([-7, -7]))
#     model1.set_pose(s1)

#     s2 = np.array([0, 0])
#     g2 = np.array([np.pi/4, 0])
#     model2 = PlanarArm(np.array([2, 2]), base_pos=np.array([-7, 0]))
#     model2.set_pose(s2)

#     s3 = np.array([0, 0])
#     g3 = np.array([-np.pi/2, 0])
#     model3 = PlanarArm(np.array([2, 2]), base_pos=np.array([-7, 7]))
#     model3.set_pose(s3)

#     s4 = np.array([0, 0])
#     g4 = np.array([np.pi/2, 0])
#     model4 = PlanarArm(np.array([2, 2]), base_pos=np.array([0, -7]))
#     model4.set_pose(s4)

#     s5 = np.array([0, 0])
#     g5 = np.array([-np.pi, 0])
#     model5 = PlanarArm(np.array([2, 2]), base_pos=np.array([0, 0]))
#     model5.set_pose(s5)

#     s6 = np.array([0, 0])
#     g6 = np.array([-np.pi/2, 0])
#     model6 = PlanarArm(np.array([2, 2]), base_pos=np.array([0, 7]))
#     model6.set_pose(s6)

#     s7 = np.array([np.pi/4*3, 0])
#     g7 = np.array([np.pi/5*3, 0])
#     model7 = PlanarArm(np.array([2, 2]), base_pos=np.array([7, -7]))
#     model7.set_pose(s7)

#     s8 = np.array([-np.pi/2, 0])
#     g8 = np.array([-np.pi, 0])
#     model8 = PlanarArm(np.array([2, 2]), base_pos=np.array([7, 0]))
#     model8.set_pose(s8)

#     s9 = np.array([-np.pi/2, 0])
#     g9 = np.array([-np.pi/4*3, 0])
#     model9 = PlanarArm(np.array([2, 2]), base_pos=np.array([7, 7]))
#     model9.set_pose(s9)

#     agents = [
#         {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None},
#         {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None},
#         {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None},
#         {"name": "agent4", "start": s4, "goal": g4, "model": model4, "roadmap": None},
#         {"name": "agent5", "start": s5, "goal": g5, "model": model5, "roadmap": None},
#         {"name": "agent6", "start": s6, "goal": g6, "model": model6, "roadmap": None},
#         {"name": "agent7", "start": s7, "goal": g7, "model": model7, "roadmap": None},
#         {"name": "agent8", "start": s8, "goal": g8, "model": model8, "roadmap": None}, 
#         {"name": "agent9", "start": s9, "goal": g9, "model": model9, "roadmap": None}
#     ]

#     return agents

def test_planner_settings(): 
    connect(use_gui=True)
    add_data_path()
    set_camera_pose(camera_point=[0, -1.2, 1.2])
    ground = p.loadURDF("plane.urdf")

    # agents
    agents = load_scenario_n(6)

    # obstacles
    obstacles = [ground]

    # environment
    env = Environment(agents, obstacles)

    # Settings
    n_nodes = 10
    maxdist = 0.1

    # Create planner instance
    planner_type = 'CBSPRM'
    assert planner_type in ['CBSPRM', 'PrioritizedPRM'], "Invalid planner type. Choose from ['CBSPRM', 'PrioritizedPRM']"
    if planner_type == 'CBSPRM':
        prm = CBSPRM(env, maxdist, k1=10, k2=5, build_type='n', prm_type='degree', n=n_nodes, t=10, time_step=0.01, local_step=0.02)
    elif planner_type == 'PrioritizedPRM':
        prm = PrioritizedPRM(env, maxdist, k1=10, k2=5, build_type='n', prm_type='degree', n=n_nodes, t=10, time_step=0.01, local_step=0.02)

    # # change planner settings
    # while True: 
    #     print(f"\nCurrent parameters: \
    #         \nplanner_type = {planner_type} \
    #         \nn_nodes = {prm.compute_combined_nr_nodes()} \
    #         \nmax_edge_len = {prm.maxdist} \
    #         \navg_degree = {prm.compute_combined_avg_degree()}")
    #     print("\nOptions: \n1 - Add samples\n2 - Change max_edge_len\n3 - Exit")
    #     choice = input("Enter number: ")
    #     if choice=='1':
    #         n_to_add = int(input("Enter the number of samples to add: "))
    #         prm.add_and_update(n_to_add)
    #         print(f"Added {n_to_add} samples.")
    #     elif choice=='2': 
    #         new_maxdist = float(input("Enter new max_edge_len: "))
    #         prm.maxdist = new_maxdist
    #         start_time = time.perf_counter()
    #         prm.update_roadmaps_edges()
    #         learning_time = time.perf_counter() - start_time
    #         print(f"learning time: {learning_time}")
    #         print(f"Updated maxdist to {new_maxdist}.")
    #     elif choice == '3':
    #         print("Exiting parameter update...")
    #         break
    #     else:
    #         print("Invalid option, please enter an integer between 1 and 3.")
    #         continue

    paths = prm.query()
    print(f"success: {1 if paths else 0}")
    if paths: 
        pause_sim('Show paths?')
        env.execute_joint_motion(paths)
    pause_sim('Disconnect?')
    disconnect()

def generate_data():
    connect(use_gui=True)
    add_data_path()
    set_camera_pose(camera_point=[0, -1.2, 1.2])
    ground = p.loadURDF("plane.urdf")

    # agents
    agents = load_scenario_n(5)

    # obstacles
    obstacles = []

    # environment
    env = Environment(agents, obstacles)
    # env.visualize()

    # run tests
    planner_type = 'PrioritizedPRM'
    assert planner_type in ['CBSPRM', 'PrioritizedPRM'], "Invalid planner type. Choose from ['CBSPRM', 'PrioritizedPRM']"
    n_tests = 10
    n_nodes = 20  

    # Run tests and generate data
    if planner_type == 'CBSPRM':
        learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths, n_ct_nodes = run_tests(env, planner_type=planner_type, n_tests=n_tests, n_nodes=n_nodes)  
    elif planner_type == 'PrioritizedPRM':
        learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths, n_ct_nodes = run_tests(env, planner_type=planner_type, n_tests=n_tests, n_nodes=n_nodes)
    
    # print(f"learning time: {learning_times}")
    # print(f"query time: {query_times}")
    # print(f"n_nodes: {nodes_amount}")
    # print(f"n_edges: {edges_amount}")
    # print(f"avg_degree: {degrees}")
    # print(f"max_edge_len: {maxdist}")
    # print(f"success: {success}")
    # print(f"path_length: {path_lengths}")
    # print(f"n_ct_nodes: {n_ct_nodes}")

    # save data
    if planner_type == 'CBSPRM':
        data = {'learning_time': learning_times, 'query_time': query_times, 'n_nodes': nodes_amount, 'n_edges': edges_amount, 'avg_degree': degrees, 'max_edge_len': maxdist, 'success': success, 'path_length': path_lengths, 'n_ct_nodes': n_ct_nodes}
    elif planner_type == 'PrioritizedPRM':
        data = {'learning_time': learning_times, 'query_time': query_times, 'n_nodes': nodes_amount, 'n_edges': edges_amount, 'avg_degree': degrees, 'max_edge_len': maxdist, 'success': success, 'path_length': path_lengths}
    df = pd.DataFrame(data)
    print(df.head())

    # Save to CSV in a specific OneDrive folder path
    flag = input("Save data to CSV? (y/N): ")
    if flag == 'y':
        folder_path = os.path.join(os.path.dirname(__file__), '../../res/data/pybullet/exp3/')
        file_name = f'{planner_type}_{len(agents)}agents_{n_tests}tests_{n_nodes}nodesPerAgent_v2_2.csv'
        save_data_to_csv(df, folder_path, file_name)
    else: 
        print("Data not saved.")

    return

def save_data_to_csv(data_frame, folder_path, file_name):
    if not os.path.exists(folder_path):
        raise FileNotFoundError(f"Folder path {folder_path} does not exist.")
    file_path = folder_path + file_name
    data_frame.to_csv(file_path, index=False)
    print(f"Data saved to {file_path}")
    return

def run_tests(env, planner_type=None, n_tests=10, n_nodes=20):
    learning_times = []
    query_times = []
    nodes_amount = []
    edges_amount = []
    degrees = []
    maxdist = []
    success = []
    path_lengths = []
    n_ct_nodes = [] if planner_type == 'CBSPRM' else None

    for i in range(n_tests):
        print(f"Test {i+1}/{n_tests}")

        # reset robots to start
        for i, agent in enumerate(env.agents):
            env.robot_models[i].set_arm_pose(agent['start'])

        # Generate PRM instance and learn roadmap
        start_time = time.perf_counter()
        if planner_type == 'CBSPRM':
            prm = CBSPRM(env, maxdist=0.1, k1=10, k2=5, build_type='n', prm_type='degree', n=n_nodes, t=10, time_step=0.01, local_step=0.02)
        elif planner_type == 'PrioritizedPRM':
            prm = PrioritizedPRM(env, maxdist=0.1, k1=10, k2=5, build_type='n', prm_type='degree', n=n_nodes, t=10, time_step=0.01, local_step=0.02)
        learning_time = time.perf_counter() - start_time
        print(f"learning time: {learning_time}")

        # Query PRM for paths
        start_time = time.perf_counter()
        paths = prm.query()
        query_time = time.perf_counter() - start_time

        # Save data
        learning_times.append(learning_time)
        query_times.append(query_time)
        nodes_amount.append(prm.compute_combined_nr_nodes())
        edges_amount.append(prm.compute_combined_nr_edges())
        degrees.append(prm.compute_combined_avg_degree())
        maxdist.append(prm.maxdist)
        success.append(1 if paths else 0)
        print(f"success: {success[-1]}")
        path_lengths.append(prm.compute_total_path_length(paths))
        n_ct_nodes.append(prm.n_ct_nodes) if planner_type == 'CBSPRM' else None
    
    return learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths, n_ct_nodes

if __name__ == "__main__":
    # test_planner_settings()
    generate_data()