import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def plot_scatter_data():
    # Set the aesthetic style of the plots
    sns.set_theme(style="white")

    # specify folder path where data is stored
    # folder_path = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Data\scene2'
    folder_path = os.path.join(os.path.dirname(__file__), '../../res/data')

    # x-axis values for plotting: the number of agents in every experiment, 
    agents_list = [2, 3, 4, 5]  # Number of agents you have data for
    
    # Load data from data files
    data_cbs = {}
    data_priority = {}
    data_distance = {}

    # Load data for each number of agents from cbs
    for num_agents in agents_list:
        # Determine full path to read data from
        filename = f'\scene2_CBSPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        full_path = folder_path + filename
        # Read data
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
        # Determine full path to read data from
        filename = f'\scene2_PriorityPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        full_path = folder_path + filename
        # Read data
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
        # Determine full path to read data from
        filename = f'\scene2_PriorityPRM_{num_agents}agents_10tests_200nodesPerAgent.csv'
        full_path = folder_path + filename
        # Read data
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

if __name__ == '__main__':
    plot_scatter_data()