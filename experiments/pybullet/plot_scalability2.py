import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def plot_line_data():
    # Set the aesthetic style of the plots
    sns.set_theme(style="whitegrid")

    # specify folder path where data is stored
    folder_path = os.path.join(os.path.dirname(__file__), '../../res/data/pybullet/exp3/')

    # x-axis values for plotting: the number of agents in every experiment
    # agents_list = [2, 3, 4, 5, 6, 7]  # Number of agents you have data for
    agents_list = [2, 4, 6, 8, 10]  # Number of agents you have data for

    # Load data from data files
    data_cbs = {}
    data_priority = {}

    # Load data for each number of agents from cbs
    for num_agents in agents_list:
        # Determine full path to read data from
        filename = f'CBSPRM_{num_agents}agents_duos.csv'
        full_path = os.path.join(folder_path, filename)
        # Read data
        if os.path.exists(full_path):
            df = pd.read_csv(full_path)
            df['Number of Agents'] = num_agents  # Add a column for grouping by number of agents
            df['Algorithm'] = 'CBS PRM'
            data_cbs[num_agents] = df
        else:
            print(f"File not found: {full_path}")

    # Load data for each number of agents from priority
    for num_agents in agents_list:
        # Determine full path to read data from
        filename = f'PrioritizedPRM_{num_agents}agents_duos.csv'
        full_path = os.path.join(folder_path, filename)
        # Read data
        if os.path.exists(full_path):
            df = pd.read_csv(full_path)
            df['Number of Agents'] = num_agents  # Add a column for grouping by number of agents
            df['Algorithm'] = 'Prioritized PRM'
            data_priority[num_agents] = df
        else:
            print(f"File not found: {full_path}")

    # Concatenate all dataframes for easier plotting
    combined_data_cbs = pd.concat(data_cbs.values(), ignore_index=True)
    combined_data_priority = pd.concat(data_priority.values(), ignore_index=True)
    combined_data = pd.concat([combined_data_cbs, combined_data_priority], ignore_index=True)

    # Calculate mean values for each number of agents and algorithm
    info = 'learning_time'
    mean_values = combined_data.groupby(['Number of Agents', 'Algorithm'])[info].mean().reset_index()

    # Plotting the mean values with lines connecting them
    plt.figure(figsize=(8, 6))
    sns.lineplot(data=mean_values, x='Number of Agents', y=info, hue='Algorithm', marker='o', palette=['blue', 'red'])

    # Set plot labels and title
    plt.xlabel("Number of Robots")
    plt.ylabel("Average Learning Time [s]")
    # plt.ylim(0, 80)  # Set y-axis limit
    # plt.title("Average Query Time vs Number of Robots")

    # Ensure only specified agent numbers are on the x-axis
    plt.xticks(agents_list)
    plt.xlim(min(agents_list) - 1, max(agents_list) + 1)  # Optional: Slightly adjust x-axis limits for clarity
    plt.tight_layout()  # Adjust layout to not cut off labels

    # Show the plot
    plt.show()

if __name__ == '__main__':
    plot_line_data()
    if input('Save plot as EPS? (y/N)') == 'y':
        name = input('Enter filename: ')
        directory = r'C:\Users\vikto\OneDrive - Vrije Universiteit Brussel\VUB\Thesis\Pictures'
        plt.savefig(os.path.join(directory, f'/{name}.eps'), format='eps', dpi=300)
    else:
        print('Plot not saved.')
    plt.close()