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

from robots.planar_arm import PlanarArm
from planners.prm.planar.env import Environment
from planners.prm.planar.multi_arm.decoupled.prioritized_prm import PrioritizedPRM
from planners.prm.planar.multi_arm.decoupled.cbs_prm import CBSPRM

def load_scenario_1(): 
    s1 = np.array([-np.pi/4*3, 0, 0])
    g1 = np.array([-np.pi/4, 0, 0])
    model1 = PlanarArm(np.array([2, 2, 2]), base_pos=np.array([0, 5]))
    model1.set_pose(s1)

    s2 = np.array([np.pi/4, 0, 0])
    g2 = np.array([np.pi/4*3, 0, 0])
    model2 = PlanarArm(np.array([2, 2, 2]), base_pos=np.array([0, -5]))
    model2.set_pose(s2)

    agents = [            
            {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None},
            {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}
            ]
    
    return agents

def load_scenario_2():
    s1 = np.array([0, 0])
    g1 = np.array([np.pi/3, 0])
    model1 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5.6, -4]))
    model1.set_pose(s1)

    s2 = np.array([np.pi/3*2, 0])
    g2 = np.array([np.pi, 0])
    model2 = PlanarArm(np.array([3, 3]), base_pos=np.array([5.6, -4]))
    model2.set_pose(s2)

    s3 = np.array([-np.pi/3*2, 0])
    g3 = np.array([-np.pi/3, 0])
    model3 = PlanarArm(np.array([3, 3]), base_pos=np.array([0, 4.6]))
    model3.set_pose(s3)
    
    agents = [
            {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None}, 
            {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}, 
            {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None}        
            ]
    
    return agents

def load_scenario_3():
    s1 = np.array([0,0])
    g1 = np.array([np.pi/2,0])
    model1 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5, -5]))
    model1.set_pose(s1)

    s2 = np.array([-np.pi/2, 0])
    g2 = np.array([0, 0])
    model2 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5, 5]))
    model2.set_pose(s2)

    s3 = np.array([-np.pi, 0])
    g3 = np.array([-np.pi/2, 0])
    model3 = PlanarArm(np.array([3, 3]), base_pos=np.array([5, 5]))
    model3.set_pose(s3)

    s4 = np.array([np.pi/2, 0])
    g4 = np.array([np.pi, 0])
    model4 = PlanarArm(np.array([3, 3]), base_pos=np.array([5, -5]))
    model4.set_pose(s4)
    
    agents = [
            {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None}, 
            {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}, 
            {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None},  
            {"name": "agent4", "start": s4, "goal": g4, "model": model4, "roadmap": None}              
            ]
    
    return agents

def load_scenario_4():
    s1 = np.array([np.pi/5*3, 0])
    g1 = np.array([0, 0])
    model1 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5, -7]))
    model1.set_pose(s1)

    s2 = np.array([np.pi/5, 0])
    g2 = np.array([-np.pi/5*2, 0])
    model2 = PlanarArm(np.array([3, 3]), base_pos=np.array([-7.5, 1.5]))
    model2.set_pose(s2)

    s3 = np.array([-np.pi/5, 0])
    g3 = np.array([-np.pi/5*4, 0])
    model3 = PlanarArm(np.array([3, 3]), base_pos=np.array([0, 7]))
    model3.set_pose(s3)

    s4 = np.array([-np.pi/5*3, 0])
    g4 = np.array([-np.pi, 0])
    model4 = PlanarArm(np.array([3, 3]), base_pos=np.array([7.5, 1.5]))
    model4.set_pose(s4)

    s5 = np.array([np.pi, 0])
    g5 = np.array([np.pi/5*2, 0])
    model5 = PlanarArm(np.array([3, 3]), base_pos=np.array([5, -7]))
    model5.set_pose(s5)
    
    agents = [
            {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None}, 
            {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}, 
            {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None},  
            {"name": "agent4", "start": s4, "goal": g4, "model": model4, "roadmap": None}, 
            {"name": "agent5", "start": s5, "goal": g5, "model": model5, "roadmap": None}             
            ]
    
    return agents

def load_scenario_5():
    s1 = np.array([0, 0])
    g1 = np.array([np.pi/4, 0])
    model1 = PlanarArm(np.array([2, 2]), base_pos=np.array([-7, -7]))
    model1.set_pose(s1)

    s2 = np.array([0, 0])
    g2 = np.array([np.pi/4, 0])
    model2 = PlanarArm(np.array([2, 2]), base_pos=np.array([-7, 0]))
    model2.set_pose(s2)

    s3 = np.array([0, 0])
    g3 = np.array([-np.pi/2, 0])
    model3 = PlanarArm(np.array([2, 2]), base_pos=np.array([-7, 7]))
    model3.set_pose(s3)

    s4 = np.array([0, 0])
    g4 = np.array([np.pi/2, 0])
    model4 = PlanarArm(np.array([2, 2]), base_pos=np.array([0, -7]))
    model4.set_pose(s4)

    s5 = np.array([0, 0])
    g5 = np.array([-np.pi, 0])
    model5 = PlanarArm(np.array([2, 2]), base_pos=np.array([0, 0]))
    model5.set_pose(s5)

    s6 = np.array([0, 0])
    g6 = np.array([-np.pi/2, 0])
    model6 = PlanarArm(np.array([2, 2]), base_pos=np.array([0, 7]))
    model6.set_pose(s6)

    s7 = np.array([np.pi/4*3, 0])
    g7 = np.array([np.pi/5*3, 0])
    model7 = PlanarArm(np.array([2, 2]), base_pos=np.array([7, -7]))
    model7.set_pose(s7)

    s8 = np.array([-np.pi/2, 0])
    g8 = np.array([-np.pi, 0])
    model8 = PlanarArm(np.array([2, 2]), base_pos=np.array([7, 0]))
    model8.set_pose(s8)

    s9 = np.array([-np.pi/2, 0])
    g9 = np.array([-np.pi/4*3, 0])
    model9 = PlanarArm(np.array([2, 2]), base_pos=np.array([7, 7]))
    model9.set_pose(s9)

    agents = [
        {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None},
        {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None},
        {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None},
        {"name": "agent4", "start": s4, "goal": g4, "model": model4, "roadmap": None},
        {"name": "agent5", "start": s5, "goal": g5, "model": model5, "roadmap": None},
        {"name": "agent6", "start": s6, "goal": g6, "model": model6, "roadmap": None},
        {"name": "agent7", "start": s7, "goal": g7, "model": model7, "roadmap": None},
        {"name": "agent8", "start": s8, "goal": g8, "model": model8, "roadmap": None}, 
        {"name": "agent9", "start": s9, "goal": g9, "model": model9, "roadmap": None}
    ]

    return agents

def test_planner_settings(): 
    agents = load_scenario_1()

    # obstacles
    obstacles = []

    # environment
    env = Environment([20, 20], agents, obstacles)
    env.visualize()

    # Settings
    n_nodes = 200
    max_edge_len = 1.5

    # Create planner instance
    prm = CBSPRM(env, max_edge_len=max_edge_len, build_type='n_nodes', n_nodes=n_nodes)
    prm.plot_roadmap_2D_subspace(0)
    prm.plot_roadmap_2D_subspace(1)

    # change planner settings
    while True: 
        print(f"\nCurrent parameters: \
            \nn_nodes = {prm.compute_combined_nr_nodes()} \
            \nmax_edge_len = {prm.max_edge_len} \
            \navg_degree = {prm.compute_combined_avg_degree()}")
        print("\nOptions: \n1 - Add samples\n2 - Change max_edge_len\n3 - Exit")
        choice = input("Enter number: ")
        if choice=='1':
            n_to_add = int(input("Enter the number of samples to add: "))
            prm.add_and_update(n_to_add)
            print(f"Added {n_to_add} samples.")
        elif choice=='2': 
            new_len = float(input("Enter new max_edge_len: "))
            prm.max_edge_len = new_len
            start_time = time.perf_counter()
            prm.update_roadmap()
            learning_time = time.perf_counter() - start_time
            print(f"learning time: {learning_time}")
            print(f"Updated max_edge_len to {new_len}.")
        elif choice == '3':
            print("Exiting parameter update...")
            break
        else:
            print("Invalid option, please enter an integer between 1 and 3.")
            continue

    paths = prm.query()
    print(f"success: {1 if paths else 0}")
    # env.execute_decoupled_interpolated_motion(paths)

def generate_data():
    agents = load_scenario_1()

    # obstacles
    obstacles = []

    # environment
    env = Environment([20, 20], agents, obstacles)
    # env.visualize()

    # run tests
    planner_type = 'CBSPRM'
    assert planner_type in ['CBSPRM', 'PrioritizedPRM'], "Invalid planner type. Choose from ['CBSPRM', 'PrioritizedPRM']"
    n_tests = 2
    n_nodes = 200  

    # Run tests and generate data
    if planner_type == 'CBSPRM':
        learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths, n_ct_nodes = run_tests(env, planner_type=planner_type, n_tests=n_tests, n_nodes=n_nodes)  
    elif planner_type == 'PrioritizedPRM':
        learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths = run_tests(env, planner_type=planner_type, n_tests=n_tests, n_nodes=n_nodes)
    
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
        folder_path = os.path.join(os.path.dirname(__file__), '../../res/data/planar/exp3/')
        file_name = f'{planner_type}_{len(agents)}agents_{n_tests}tests_{n_nodes}nodesPerAgent.csv'
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

def run_tests(env, planner_type=None, n_tests=10, n_nodes=200):
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
            env.robot_models[i].set_pose(agent['start'])

        # Generate PRM instance and learn roadmap
        start_time = time.perf_counter()
        if planner_type == 'CBSPRM':
            prm = CBSPRM(env, max_edge_len=1.5, build_type='n_nodes', n_nodes=n_nodes)
        elif planner_type == 'PrioritizedPRM':
            prm = PrioritizedPRM(env, max_edge_len=1.5, build_type='n_nodes', n_nodes=n_nodes)
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
        maxdist.append(prm.max_edge_len)
        success.append(1 if paths else 0)
        print(f"success: {success[-1]}")
        path_lengths.append(prm.compute_total_path_length(paths))
        n_ct_nodes.append(prm.n_ct_nodes) if planner_type == 'CBSPRM' else None
    
    return learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths, n_ct_nodes

if __name__ == "__main__":
    # test_planner_settings()
    generate_data()