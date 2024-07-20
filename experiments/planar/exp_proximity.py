import time
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Rectangle, Circle
# Define the path to your project root directory
project_root = 'C:\\Users\\vikto\\Thesis'

# Add the project root directory to sys.path if it's not already included
import sys
import os
if project_root not in sys.path:
    sys.path.insert(0, project_root)
from motion_planners.prm.cprmPA import PlanarArm
from motion_planners.prm.cprmPA import Environment
from motion_planners.prm.cprmPA import DistanceCoupledPRM, KDTreeDistanceCoupledPRM, KDTreeKNNCoupledPRM, PrioritizedPRM, CBSPRM

def main():
    # agents
    start1 = np.array([np.pi/2, -np.pi/16])
    goal1 = np.array([0, 0])
    model1 = PlanarArm(np.array([2, 2]), base_pos=np.array([-5, -5]))
    model1.set_pose(start1)

    start2 = np.array([-np.pi/2, np.pi/6])
    goal2 = np.array([0, 0])
    model2 = PlanarArm(np.array([2, 2]), base_pos=np.array([-5, 5]))
    model2.set_pose(start2)

    start3 = np.array([0, np.pi/12])
    goal3 = np.array([0, 0])
    model3 = PlanarArm(np.array([2, 2]), base_pos=np.array([5, 5]))
    model3.set_pose(goal3)

    start4 = np.array([np.pi/8*7, -np.pi/3])
    goal4 = np.array([0, 0])
    model4 = PlanarArm(np.array([2, 2]), base_pos=np.array([5, -5]))
    model4.set_pose(goal4)
    
    agents = [
            {"name": "agent1", "start": start1, "goal": goal1, "model": model1, "roadmap": None}, 
            {"name": "agent2", "start": start2, "goal": goal2, "model": model2, "roadmap": None}, 
            {"name": "agent3", "start": start3, "goal": goal3, "model": model3, "roadmap": None}, 
            {"name": "agent4", "start": start4, "goal": goal4, "model": model4, "roadmap": None}           
        ]

    # obstacles
    obstacles = []

    # environment
    env = Environment([20, 20], agents, obstacles)
    # env.visualize()

    # Generate PRM instance
    # prm = DistanceCoupledPRM(env, max_edge_len=2.5, build_type='n_nodes', n_nodes=0)
    # prm = PrioritizedPRM(env, max_edge_len=1, build_type='n_nodes', n_nodes=0)
    # prm = CBSPRM(env, max_edge_len=1, build_type='n_nodes', n_nodes=0)

    # run tests
    n_tests = 10
    n_nodes = 200  
    learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths = run_tests(env, n_tests=n_tests, n_nodes=n_nodes)

    print(f"learning time: {learning_times}")
    print(f"query time: {query_times}")
    print(f"n_nodes: {nodes_amount}")
    print(f"n_edges: {edges_amount}")
    print(f"avg_degree: {degrees}")
    print(f"max_edge_len: {maxdist}")
    print(f"success: {success}")
    print(f"path_length: {path_lengths}")

    # save data
    data = {'learning_time': learning_times, 'query_time': query_times, 'n_nodes': nodes_amount, 'n_edges': edges_amount, 'avg_degree': degrees, 'max_edge_len': maxdist, 'success': success, 'path_length': path_lengths}
    df = pd.DataFrame(data)
    # Save to CSV in a specific OneDrive folder path
    onedrive_path = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Data\scene1'
    filename = f'\scene1_CBSPRM_{len(agents)}agents_{n_tests}tests_{n_nodes}nodes.csv'
    # df.to_csv(onedrive_path + filename, index=False)


def plot_results(): 
    # Step 1: Load data from a CSV file
    folder_path = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Data\scene1'
    filename = '\scene1_CBSPRM_10tests_200nodes.csv'
    data = pd.read_csv(folder_path + filename)

    # Step 2: Check the first few rows of the dataframe
    print(data.head())

    # Assuming the data has columns 'Date' and 'Value' you want to plot
    plt.figure(figsize=(10, 5))
    plt.plot(data['Date'], data['Value'], marker='o')  # Plot 'Date' vs 'Value'
    plt.title('Date vs Value')
    plt.xlabel('Date')
    plt.ylabel('Value')
    plt.grid(True)
    plt.xticks(rotation=45)  # Rotate date labels for better readability
    plt.tight_layout()  # Adjust layout to not cut off labels
    plt.show()
    

    # prm.plot_roadmap_2D_subspace(0)

def run_tests(env, n_tests=10, n_nodes=200):
    learning_times = []
    query_times = []
    nodes_amount = []
    edges_amount = []
    degrees = []
    maxdist = []
    success = []
    path_lengths = []

    for i in range(n_tests):
        print(f"Test {i+1}/{n_tests}")
        # Generate PRM instance
        start_time = time.perf_counter()
        # prm = DistanceCoupledPRM(env, max_edge_len=1, build_type='n_nodes', n_nodes=n_nodes)
        # prm = PrioritizedPRM(env, max_edge_len=1, build_type='n_nodes', n_nodes=int(n_nodes/len(env.agents)))
        prm = CBSPRM(env, max_edge_len=1, build_type='n_nodes', n_nodes=int(n_nodes/len(env.agents)))
        learning_time = time.perf_counter() - start_time
        print(f"learning time: {learning_time}")

        # change roadmap parameter max_edge_len or n_knn
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
            
        # plan motion
        start_time = time.perf_counter()
        paths = prm.query()
        query_time = time.perf_counter() - start_time

        learning_times.append(learning_time)
        query_times.append(query_time)
        nodes_amount.append(prm.compute_combined_nr_nodes())
        edges_amount.append(prm.compute_combined_nr_edges())
        degrees.append(prm.compute_combined_avg_degree())
        maxdist.append(prm.max_edge_len)
        success.append(1 if paths else 0)
        path_lengths.append(prm.compute_total_path_length(paths))
    
    return learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths

    print(f"learning time: {learning_times}")
    print(f"query time: {query_times}")
    print(f"n_nodes: {nodes_amount}")
    print(f"n_edges: {edges_amount}")
    print(f"avg_degree: {degrees}")
    print(f"max_edge_len: {maxdist}")
    print(f"collision_checks: {collision_checks}")
    print(f"success: {success}")
    print(f"path_length: {path_lengths}")

def run_degree_query_time_test(agents, obstacles, n_tests=10, n_nodes=100):
    degrees = []
    query_times = []
    for i in range(n_tests):
        print(f"Test {i+1}/{n_tests}")
        # environment
        env = Environment([20, 20], agents, obstacles)
        # env.visualize()

        # Generate PRM instance
        prm = PrioritizedPRM(env, max_edge_len=1, build_type='n_nodes', n_nodes=n_nodes)

        # change roadmap parameter max_edge_len or n_knn
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
                prm.update_roadmap()
                print(f"Updated max_edge_len to {new_len}.")
            elif choice == '3':
                print("Exiting parameter update...")
                break
            else:
                print("Invalid option, please enter an integer between 1 and 3.")
                continue
            
        # plan motion
        start_time = time.perf_counter()
        paths = prm.query()
        query_time = time.perf_counter() - start_time
        if paths == {}:
            query_time = 0

        degrees.append(prm.compute_combined_avg_degree())
        print(prm.compute_combined_avg_degree())
        query_times.append(query_time)
        print(query_time)
    return degrees, query_times

def run_scalability_test(agents, obstacles, n_tests=10, n_nodes=100):
    learning_times = []
    query_times = []
    nodes_amount = []
    edges_amount = []
    degrees = []
    maxdist = []
    collision_checks = []
    success = []
    path_lengths = []

    for i in range(n_tests):
        print(f"Test {i+1}/{n_tests}")
        # environment
        env = Environment([20, 20], agents, obstacles)
        # env.visualize()

        # Generate PRM instance
        start_time = time.perf_counter()
        # prm = DistanceCoupledPRM(env, max_edge_len=1, build_type='n_nodes', n_nodes=n_nodes)
        prm = PrioritizedPRM(env, max_edge_len=1, build_type='n_nodes', n_nodes=n_nodes)
        # prm = CBSPRM(env, max_edge_len=1, build_type='n_nodes', n_nodes=n_nodes)
        learning_time = time.perf_counter() - start_time
        print(f"learning time: {learning_time}")

        # change roadmap parameter max_edge_len or n_knn
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
            
        # plan motion
        start_time = time.perf_counter()
        paths = prm.query()
        query_time = time.perf_counter() - start_time

        learning_times.append(learning_time)
        query_times.append(query_time)
        nodes_amount.append(prm.compute_combined_nr_nodes())
        edges_amount.append(prm.compute_combined_nr_edges())
        degrees.append(prm.compute_combined_avg_degree())
        maxdist.append(prm.max_edge_len)
        collision_checks.append(prm.collision_checks)
        success.append(1 if paths else 0)
        path_lengths.append(prm.compute_total_path_length(paths))

    print(f"learning time: {learning_times}")
    print(f"query time: {query_times}")
    print(f"n_nodes: {nodes_amount}")
    print(f"n_edges: {edges_amount}")
    print(f"avg_degree: {degrees}")
    print(f"max_edge_len: {maxdist}")
    print(f"collision_checks: {collision_checks}")
    print(f"success: {success}")
    print(f"path_length: {path_lengths}")

    # # save data
    # data = {'PRM': {'learning_time': learning_times["PRM"], 'query_time': query_times["PRM"], 'n_nodes': nodes_amount["PRM"], 'n_edges': edges_amount["PRM"], 'avg_degree': degrees["PRM"], 'max_edge_len': maxdist["PRM"], 'collision_checks': collision_checks["PRM"], 'success': success["PRM"], 'path_length': path_lengths["PRM"]},
    #         'Priority': {'learning_time': learning_times["Priority"], 'query_time': query_times["Priority"], 'n_nodes': nodes_amount["Priority"], 'n_edges': edges_amount["Priority"], 'avg_degree': degrees["Priority"], 'max_edge_len': maxdist["Priority"], 'collision_checks': collision_checks["Priority"], 'success': success["Priority"], 'path_length': path_lengths["Priority"]},
    #         'CBS': {'learning_time': learning_times["CBS"], 'query_time': query_times["CBS"], 'n_nodes': nodes_amount["CBS"], 'n_edges': edges_amount["CBS"], 'avg_degree': degrees["CBS"], 'max_edge_len': maxdist["CBS"], 'collision_checks': collision_checks["CBS"], 'success': success["CBS"], 'path_length': path_lengths["CBS"]}}

    # df = pd.DataFrame(data)
    # # Save to CSV in a specific OneDrive folder path
    # onedrive_path = "C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Data\scene1"
    # filename = f'scene1_scal_{n_tests}tests_{len(agents)}agents_{n_nodes}nodes.csv'
    # df.to_csv(onedrive_path + filename, index=False)

if __name__ == "__main__":
    plot_results()
    # main()