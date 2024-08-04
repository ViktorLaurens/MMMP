import os

import pandas as pd


def get_mean_table():
    # Define the folder path and file pattern
    folder_path = os.path.join(os.path.dirname(__file__), '../../res/data/pybullet/exp3/')
    
    # List of different numbers of agents you have data for
    agents_list = [2, 3, 4, 5]
    
    # Dictionary to store mean data
    mean_data = {}

    # Iterate over each number of agents to load and process each file
    planner_type = 'CBSPRM'
    n_tests = 10
    n_nodes = 20
    for num_agents in agents_list:
        filename = f'{planner_type}_{num_agents}agents_{n_tests}tests_{n_nodes}nodesPerAgent_v2.csv'
        
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
    mean_df.index.name = 'Number of Panda\'s'
    mean_df.reset_index(inplace=True)  # If you want 'Number of Agents' as a column

    # Print the DataFrame to see the result
    print(mean_df)

    # Optionally, return the DataFrame if you want to use it outside the function
    return mean_df

if __name__ == '__main__':
    mean_table = get_mean_table()