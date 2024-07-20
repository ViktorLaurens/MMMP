import time
from matplotlib import pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Circle
import pandas as pd
import seaborn as sns
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
    # # configuration 1
    # s1 = np.array([-np.pi/4*3, 0, 0])
    # g1 = np.array([-np.pi/4, 0, 0])
    # model1 = PlanarArm(np.array([2, 2, 2]), base_pos=np.array([0, 5]))
    # model1.set_pose(s1)

    # s2 = np.array([np.pi/4, 0, 0])
    # g2 = np.array([np.pi/4*3, 0, 0])
    # model2 = PlanarArm(np.array([2, 2, 2]), base_pos=np.array([0, -5]))
    # model2.set_pose(s2)

    # agents = [            
    #         {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None},
    #         {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}
    #         ]

    # # configuration 2
    # s1 = np.array([0, 0])
    # g1 = np.array([np.pi/3, 0])
    # model1 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5.6, -4]))
    # model1.set_pose(s1)

    # s2 = np.array([np.pi/3*2, 0])
    # g2 = np.array([np.pi, 0])
    # model2 = PlanarArm(np.array([3, 3]), base_pos=np.array([5.6, -4]))
    # model2.set_pose(s2)

    # s3 = np.array([-np.pi/3*2, 0])
    # g3 = np.array([-np.pi/3, 0])
    # model3 = PlanarArm(np.array([3, 3]), base_pos=np.array([0, 4.6]))
    # model3.set_pose(s3)
    
    # agents = [
    #         {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None}, 
    #         {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}, 
    #         {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None}        
    #         ]
    
    # configuration 3
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
    
    # # configuration 4
    # s1 = np.array([np.pi/5*3, 0])
    # g1 = np.array([0, 0])
    # model1 = PlanarArm(np.array([3, 3]), base_pos=np.array([-5, -7]))
    # model1.set_pose(s1)

    # s2 = np.array([np.pi/5, 0])
    # g2 = np.array([-np.pi/5*2, 0])
    # model2 = PlanarArm(np.array([3, 3]), base_pos=np.array([-7.5, 1.5]))
    # model2.set_pose(s2)

    # s3 = np.array([-np.pi/5, 0])
    # g3 = np.array([-np.pi/5*4, 0])
    # model3 = PlanarArm(np.array([3, 3]), base_pos=np.array([0, 7]))
    # model3.set_pose(s3)

    # s4 = np.array([-np.pi/5*3, 0])
    # g4 = np.array([-np.pi, 0])
    # model4 = PlanarArm(np.array([3, 3]), base_pos=np.array([7.5, 1.5]))
    # model4.set_pose(s4)

    # s5 = np.array([np.pi, 0])
    # g5 = np.array([np.pi/5*2, 0])
    # model5 = PlanarArm(np.array([3, 3]), base_pos=np.array([5, -7]))
    # model5.set_pose(s5)
    
    # agents = [
    #         {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None}, 
    #         {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None}, 
    #         {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None},  
    #         {"name": "agent4", "start": s4, "goal": g4, "model": model4, "roadmap": None}, 
    #         {"name": "agent5", "start": s5, "goal": g5, "model": model5, "roadmap": None}             
    #         ]

    # # configuration 5
    # s1 = np.array([0, 0])
    # g1 = np.array([np.pi/4, 0])
    # model1 = PlanarArm(np.array([2, 2]), base_pos=np.array([-7, -7]))
    # model1.set_pose(s1)

    # s2 = np.array([0, 0])
    # g2 = np.array([np.pi/4, 0])
    # model2 = PlanarArm(np.array([2, 2]), base_pos=np.array([-7, 0]))
    # model2.set_pose(s2)

    # s3 = np.array([0, 0])
    # g3 = np.array([-np.pi/2, 0])
    # model3 = PlanarArm(np.array([2, 2]), base_pos=np.array([-7, 7]))
    # model3.set_pose(s3)

    # s4 = np.array([0, 0])
    # g4 = np.array([np.pi/2, 0])
    # model4 = PlanarArm(np.array([2, 2]), base_pos=np.array([0, -7]))
    # model4.set_pose(s4)

    # s5 = np.array([0, 0])
    # g5 = np.array([-np.pi, 0])
    # model5 = PlanarArm(np.array([2, 2]), base_pos=np.array([0, 0]))
    # model5.set_pose(s5)

    # s6 = np.array([0, 0])
    # g6 = np.array([-np.pi/2, 0])
    # model6 = PlanarArm(np.array([2, 2]), base_pos=np.array([0, 7]))
    # model6.set_pose(s6)

    # s7 = np.array([np.pi/4*3, 0])
    # g7 = np.array([np.pi/5*3, 0])
    # model7 = PlanarArm(np.array([2, 2]), base_pos=np.array([7, -7]))
    # model7.set_pose(s7)

    # s8 = np.array([-np.pi/2, 0])
    # g8 = np.array([-np.pi, 0])
    # model8 = PlanarArm(np.array([2, 2]), base_pos=np.array([7, 0]))
    # model8.set_pose(s8)

    # s9 = np.array([-np.pi/2, 0])
    # g9 = np.array([-np.pi/4*3, 0])
    # model9 = PlanarArm(np.array([2, 2]), base_pos=np.array([7, 7]))
    # model9.set_pose(s9)

    # agents = [
    #     {"name": "agent1", "start": s1, "goal": g1, "model": model1, "roadmap": None},
    #     {"name": "agent2", "start": s2, "goal": g2, "model": model2, "roadmap": None},
    #     {"name": "agent3", "start": s3, "goal": g3, "model": model3, "roadmap": None},
    #     {"name": "agent4", "start": s4, "goal": g4, "model": model4, "roadmap": None},
    #     {"name": "agent5", "start": s5, "goal": g5, "model": model5, "roadmap": None},
    #     {"name": "agent6", "start": s6, "goal": g6, "model": model6, "roadmap": None},
    #     {"name": "agent7", "start": s7, "goal": g7, "model": model7, "roadmap": None},
    #     {"name": "agent8", "start": s8, "goal": g8, "model": model8, "roadmap": None}, 
    #     {"name": "agent9", "start": s9, "goal": g9, "model": model9, "roadmap": None}
    # ]

    # obstacles
    obstacles = []

    # environment
    # print(model1.pos)
    # print(model2.pos)
    env = Environment([20, 20], agents, obstacles)
    # env.visualize()

    # Generate PRM instance
    # prm = DistanceCoupledPRM(env, max_edge_len=2.5, build_type='n')
    # prm = CBSPRM(env, max_edge_len=1)
    # prm = KDTreeKNNCoupledPRM(env, max_edge_len=2.5, n_knn=5)

    # run tests
    n_tests = 10
    n_nodes = 200  
    # learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths, n_ct_nodes = run_tests(env, n_tests=n_tests, n_nodes=n_nodes)

    prm = CBSPRM(env, max_edge_len=0.7, build_type='n_nodes', n_nodes=n_nodes)
    print(f"degree: {prm.compute_combined_avg_degree()}")
    # prm.plot_roadmap_2D_subspace(0)
    # prm.plot_roadmap_2D_subspace(1)

    paths = prm.query()
    env.execute_decoupled_interpolated_motion(paths)

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
    # data = {'learning_time': learning_times, 'query_time': query_times, 'n_nodes': nodes_amount, 'n_edges': edges_amount, 'avg_degree': degrees, 'max_edge_len': maxdist, 'success': success, 'path_length': path_lengths, 'n_ct_nodes': n_ct_nodes}
    # df = pd.DataFrame(data)
    # # Save to CSV in a specific OneDrive folder path
    # onedrive_path = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Data\scene2'
    # filename = f'\scene2_sweep_5links_CBSPRM_{len(agents)}agents_{n_tests}tests_{n_nodes}nodesPerAgent.csv'
    # df.to_csv(onedrive_path + filename, index=False)

def run_tests(env, n_tests=10, n_nodes=200):
    learning_times = []
    query_times = []
    nodes_amount = []
    edges_amount = []
    degrees = []
    maxdist = []
    success = []
    path_lengths = []
    n_ct_nodes = []

    for i in range(n_tests):
        print(f"Test {i+1}/{n_tests}")

        # reset robots to start
        for i, agent in enumerate(env.agents):
            env.robot_models[i].set_pose(agent['start'])

        # Generate PRM instance
        start_time = time.perf_counter()
        # prm = DistanceCoupledPRM(env, max_edge_len=1.5, build_type='n_nodes', n_nodes=n_nodes)
        # prm = PrioritizedPRM(env, max_edge_len=2.8, build_type='n_nodes', n_nodes=n_nodes)
        prm = CBSPRM(env, max_edge_len=2.8, build_type='n_nodes', n_nodes=n_nodes)
        learning_time = time.perf_counter() - start_time
        print(f"learning time: {learning_time}")

        # change roadmap parameter max_edge_len or n_knn
        # while True: 
        #     print(f"\nCurrent parameters: \
        #         \nn_nodes = {prm.compute_combined_nr_nodes()} \
        #         \nmax_edge_len = {prm.max_edge_len} \
        #         \navg_degree = {prm.compute_combined_avg_degree()}")
        #     print("\nOptions: \n1 - Add samples\n2 - Change max_edge_len\n3 - Exit")
        #     choice = input("Enter number: ")
        #     if choice=='1':
        #         n_to_add = int(input("Enter the number of samples to add: "))
        #         prm.add_and_update(n_to_add)
        #         print(f"Added {n_to_add} samples.")
        #     elif choice=='2': 
        #         new_len = float(input("Enter new max_edge_len: "))
        #         prm.max_edge_len = new_len
        #         start_time = time.perf_counter()
        #         prm.update_roadmap()
        #         learning_time = time.perf_counter() - start_time
        #         print(f"learning time: {learning_time}")
        #         print(f"Updated max_edge_len to {new_len}.")
        #     elif choice == '3':
        #         print("Exiting parameter update...")
        #         break
        #     else:
        #         print("Invalid option, please enter an integer between 1 and 3.")
        #         continue
            
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
        print(f"success: {success[-1]}")
        path_lengths.append(prm.compute_total_path_length(paths))
        n_ct_nodes.append(prm.n_ct_nodes)
        # n_ct_nodes.append(0)
    
    return learning_times, query_times, nodes_amount, edges_amount, degrees, maxdist, success, path_lengths, n_ct_nodes


def plot_scalability():
    # Step 1: Load data from a CSV file
    folder_path = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Data\scene2'
    agents_list = [2, 3, 4, 5, 9]  # Number of agents you have data for
    data = {}
    
    # Load data for each number of agents
    for num_agents in agents_list:
        filename = f'\scene2_CBSPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        full_path = folder_path + filename
        if os.path.exists(full_path):
            data[num_agents] = pd.read_csv(full_path)
        else:
            print(f"File not found: {full_path}")

    # Step 2: Initialize plot
    plt.figure(figsize=(5, 5))

    # Plotting each point
    for num_agents, df in data.items():
        # Scatter all learning times in blue
        plt.scatter([num_agents] * len(df), df['learning_time'], color='blue', alpha=0.5)

    # Plotting mean values and connecting them with a line in red
    mean_values = []
    agent_numbers = sorted(data.keys())
    for num_agents in agent_numbers:
        mean_value = data[num_agents]['learning_time'].mean()
        mean_values.append(mean_value)
        plt.scatter(num_agents, mean_value, color='red')  # Scatter mean values in red

    plt.plot(agent_numbers, mean_values, color='red')  # Connect means with a line

    plt.title('Learning Time vs. Number of Robots')
    plt.xlabel('Number of Robots')
    plt.ylabel('Learning Time (s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()  # Adjust layout to not cut off labels
    plt.show()

def plot_scatter_results():
    # Set the aesthetic style of the plots
    sns.set_theme(style="white")

    folder_path = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Data\scene2'
    # agents_list = [2, 3, 4, 5, 9]  # Number of agents you have data for
    # agents_list = [2, 3, 4, 5]
    agents_list = [4, 6, 7]
    data_cbs = {}
    data_priority = {}
    data_distance = {}

    # Load data for each number of agents from cbs
    for num_agents in agents_list:
        # filename = f'\scene2_CBSPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        # filename = f'\scene2_sweep_{num_agents}links_CBSPRM_2agents_10tests_200nodesPerAgent.csv'
        filename = f'\scene2_sweep_len{num_agents}_CBSPRM_2agents_10tests_200nodesPerAgent.csv'
        
        full_path = folder_path + filename
        if os.path.exists(full_path):
            df = pd.read_csv(full_path)
            df['Number of Agents'] = num_agents  # Add a column for grouping by number of agents
            df['Algorithm'] = 'CBS PRM'
            print(df.head())
            data_cbs[num_agents] = df
        else:
            print(f"File not found: {full_path}")

    # Load data for each number of agents from priority
    for num_agents in agents_list:
        # filename = f'\scene2_PriorityPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        # filename = f'\scene2_sweep_{num_agents}links_PriorityPRM_2agents_10tests_200nodesPerAgent.csv'
        filename = f'\scene2_sweep_len{num_agents}_PriorityPRM_2agents_10tests_200nodesPerAgent.csv'

        full_path = folder_path + filename
        if os.path.exists(full_path):
            df = pd.read_csv(full_path)
            df['Number of Agents'] = num_agents  # Add a column for grouping by number of agents
            df['Algorithm'] = 'Prioritized PRM'
            print(df.head())
            data_priority[num_agents] = df
        else:
            print(f"File not found: {full_path}")
    
    # Load data for each number of agents from distance
    for num_agents in agents_list:
        # filename = f'\scene2_PriorityPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        # filename = f'\scene2_sweep_{num_agents}links_PriorityPRM_2agents_10tests_200nodesPerAgent.csv'
        filename = f'\scene2_sweep_len{num_agents}_DistancePRM_2agents_10tests_400nodesPerAgent.csv'

        full_path = folder_path + filename
        if os.path.exists(full_path):
            df = pd.read_csv(full_path)
            df['Number of Agents'] = num_agents  # Add a column for grouping by number of agents
            df['Algorithm'] = 'Distance PRM'
            data_distance[num_agents] = df
        else:
            print(f"File not found: {full_path}")

    # Concatenate all dataframes for easier plotting
    combined_data_cbs = pd.concat(data_cbs.values())
    combined_data_priority = pd.concat(data_priority.values())
    combined_data_distance = pd.concat(data_distance.values())
    combined_data = pd.concat([combined_data_cbs, combined_data_priority, combined_data_distance])


    # Plotting individual learning times
    # plt.figure(figsize=(5, 5))
    g = sns.catplot(x="Number of Agents", y="query_time", hue="Algorithm", data=combined_data,
                    palette=['blue', 'red', 'orange'], height=5, aspect=1)  # 'swarm' to avoid overlapping
    
    # Calculate means and plot them
    # mean_values = combined_data.groupby('Number of Agents')['learning_time'].mean().reset_index()
    # sns.catplot(data=mean_values, x='Number of Agents', y='learning_time', color='red', marker='o')  # Larger markers for mean values
    # plt.scatter(mean_values['Number of Agents'].apply(lambda x: agents_list.index(x)), mean_values['learning_time'], color='red', marker='o', zorder=5)

    g.set_axis_labels("Length of Robot", "Query Time (s)")
    g.legend.set_bbox_to_anchor((2, 20))  # (0, 1) refers to the coordinates of the upper left corner of the legend box.
    g.set(ylim=(0, 20))  # Set y-axis limit
    g.set_xticklabels(agents_list)  # Ensure custom agent list appears correctly
    plt.tight_layout()  # Adjust layout to not cut off labels
    plt.legend(loc='upper left')  # By location description
    plt.show()


def get_mean_table():
    # Define the folder path and file pattern
    folder_path = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Data\scene2'
    # agents_list = [2, 3, 4, 5, 9]  # List of different numbers of agents you have data for
    # agents_list = [2, 3, 4, 5]
    agents_list = [4, 6, 7]

    # Dictionary to store mean data
    mean_data = {}

    # Iterate over each number of agents to load and process each file
    for num_agents in agents_list:
        # filename = f'\scene2_PriorityPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        # filename = f'\scene2_sweep_{num_agents}links_CBSPRM_2agents_10tests_200nodesPerAgent.csv'
        # filename = f'\scene2_sweep_len{num_agents}_PriorityPRM_2agents_10tests_200nodesPerAgent.csv'
        filename = f'\scene2_sweep_len{num_agents}_DistancePRM_2agents_10tests_400nodesPerAgent.csv'
        
        # full_path = os.path.join(folder_path, filename)  # Create the full file path
        full_path = folder_path + filename
        if os.path.exists(full_path):
            # Read the CSV file
            data = pd.read_csv(full_path)

            # Compute the mean for each metric in the file
            means = data.mean()  # This returns a Series with the mean of each column

            # Add the mean series to our dictionary, using the number of agents as the key
            mean_data[num_agents] = means
        else:
            print(f"File not found: {full_path}")
    
    # Convert the dictionary of means to a DataFrame for nicer tabular representation
    mean_df = pd.DataFrame(mean_data).T  # Transpose to have agents as rows and metrics as columns

    # Optionally, format the DataFrame index and columns
    mean_df.index.name = 'Length of Robot'
    mean_df.reset_index(inplace=True)  # If you want 'Number of Agents' as a column

    # Print the DataFrame to see the result
    print(mean_df)

    # Optionally, return the DataFrame if you want to use it outside the function
    return mean_df

def plot_bar_results():
    sns.set_theme(style="white")  # Clear the background

    folder_path = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Data\scene2'
    # agents_list = [2, 3, 4, 5, 9]  # Specify the number of agents
    agents_list = [2, 3, 4, 5]
    # agents_list = [4, 6, 7]
    data_cbs = {}
    data_priority = {}
    # data_distance = {}

    # Load data for each number of agents from cbs
    for num_agents in agents_list:
        # filename = f'\scene2_CBSPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        # filename = f'\scene2_sweep_{num_agents}links_CBSPRM_2agents_10tests_200nodesPerAgent.csv'
        filename = f'\scene2_sweep_len{num_agents}_CBSPRM_2agents_10tests_200nodesPerAgent.csv'

        full_path = folder_path + filename
        if os.path.exists(full_path):
            df = pd.read_csv(full_path)
            df['Number of Agents'] = num_agents  # Add a column for grouping by number of agents
            df['Algorithm'] = 'CBS PRM'
            data_cbs[num_agents] = df
        else:
            print(f"File not found: {full_path}")

    # Load data for each number of agents from cbs
    for num_agents in agents_list:
        # filename = f'\scene2_PriorityPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        filename = f'\scene2_sweep_{num_agents}links_PriorityPRM_2agents_10tests_200nodesPerAgent.csv'
        # filename = f'\scene2_sweep_len{num_agents}_PriorityPRM_2agents_10tests_200nodesPerAgent.csv'

        full_path = folder_path + filename
        if os.path.exists(full_path):
            df = pd.read_csv(full_path)
            df['Number of Agents'] = num_agents  # Add a column for grouping by number of agents
            df['Algorithm'] = 'Prioritized PRM'
            data_priority[num_agents] = df
        else:
            print(f"File not found: {full_path}")

    # # Load data for each number of agents from distance
    # for num_agents in agents_list:
    #     # filename = f'\scene2_PriorityPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
    #     # filename = f'\scene2_sweep_{num_agents}links_PriorityPRM_2agents_10tests_200nodesPerAgent.csv'
    #     filename = f'\scene2_sweep_len{num_agents}_DistancePRM_2agents_10tests_400nodesPerAgent.csv'

    #     full_path = folder_path + filename
    #     if os.path.exists(full_path):
    #         df = pd.read_csv(full_path)
    #         df['Number of Agents'] = num_agents  # Add a column for grouping by number of agents
    #         df['Algorithm'] = 'Distance PRM'
    #         data_distance[num_agents] = df
    #     else:
    #         print(f"File not found: {full_path}")

    # Concatenate all dataframes for easier plotting
    combined_data_cbs = pd.concat(data_cbs.values())
    combined_data_priority = pd.concat(data_priority.values())
    # combined_data_distance = pd.concat(data_distance.values())
    combined_data = pd.concat([combined_data_cbs, combined_data_priority])

    # Calculate the mean success rate for each combination of number of agents and algorithm
    combined_data['success'] = combined_data['success'].astype(float)  # Ensure data type is float for mean calculation
    mean_success = combined_data.groupby(['Number of Agents', 'Algorithm'])['success'].mean().reset_index()

    # Plotting the success rates using a bar plot
    g = sns.catplot(x="Number of Agents", y="success", hue="Algorithm", data=mean_success,
                    kind="bar", palette=['blue', 'red'], height=5, aspect=1, width=0.4)

    g.set_axis_labels("Number of Links", "Success Rate")
    g.legend.set_bbox_to_anchor((0, 0))  # (0, 1) refers to the coordinates of the upper left corner of the legend box.
    g.set(ylim=(0, 1))  # Success rate is now a proportion, so ylim goes from 0 to 1
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    plot_scatter_results()
    # plot_bar_results()
    # mean_table = get_mean_table()
    # main()