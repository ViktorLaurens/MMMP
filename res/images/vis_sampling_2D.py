import numpy as np
import matplotlib.pyplot as plt
from scipy.stats.qmc import Halton
from scipy.spatial import KDTree

# Sampling
def generate_random_samples(n):
    return np.random.rand(n, 2)

def generate_grid_samples(n):
    grid_size = int(np.sqrt(n))
    grid_x, grid_y = np.meshgrid(np.linspace(0, 1, grid_size), np.linspace(0, 1, grid_size))
    grid_points = np.vstack([grid_x.ravel(), grid_y.ravel()]).T
    noise = np.random.normal(scale=0.01, size=grid_points.shape)
    return grid_points + noise

def generate_halton_samples(n):
    sampler = Halton(d=2, scramble=False)
    return sampler.random(n)

# Roadmap construction
def construct_distance_roadmap(samples, d):
    tree = KDTree(samples)
    roadmap = []
    
    for i, point in enumerate(samples):
        # Find all points within distance d
        indices = tree.query_ball_point(point, r=d)
        for j in indices:
            if i != j:
                roadmap.append((i, j))
    
    return roadmap

def construct_degree_roadmap(samples, k):
    tree = KDTree(samples)
    roadmap = []
    
    for i, point in enumerate(samples):
        # Find k nearest neighbors
        distances, indices = tree.query(point, k=k+1)  # k+1 because it includes the point itself
        for j in indices[1:]:  # Skip the point itself
            roadmap.append((i, j))
    
    return roadmap

# File I/O
def save_samples(samples, filename):
    directory = 'res/images/'
    np.savetxt(directory + filename, samples, delimiter=',')
    print(f"Samples saved to {filename}")
    return

def load_samples(filename):
    directory = 'res/images/'
    return np.loadtxt(directory + filename, delimiter=',')

def save_roadmap(roadmap, filename):
    directory = 'res/images/'
    np.savetxt(directory + filename, roadmap, fmt='%d', delimiter=',')
    print(f"Roadmap saved to {filename}")
    return

def load_roadmap(filename):
    directory = 'res/images/'
    return np.loadtxt(directory + filename, dtype=int, delimiter=',')

# Visualization
def visualize_sampling():
    fig, ax = plt.subplots()
    
    # Number of nodes to plot per method
    n = 50
    point_size = 10  # Size of the points

    # Uniform random sampling
    # random_samples = generate_random_samples(n)
    random_samples = load_samples('random_samples.csv')
    ax.scatter(random_samples[:, 0], random_samples[:, 1], color='red', s=point_size, label='Uniform Random')

    # Grid sampling with Gaussian noise
    # grid_samples = generate_grid_samples(n)
    grid_samples = load_samples('grid_samples.csv')
    ax.scatter(grid_samples[:, 0], grid_samples[:, 1], color='blue', s=point_size, label='Grid with Gaussian Noise')

    # Halton sequence sampling (2, 3 dimensions)
    # halton_samples = generate_halton_samples(n)
    halton_samples = load_samples('halton_samples.csv')
    ax.scatter(halton_samples[:, 0], halton_samples[:, 1], color='green', s=point_size, label='Halton Sequence (2, 3)')

    # Plot settings
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.legend(loc='upper right')
    
    plt.show()

    # Save samples to file
    # if input("Save random samples? (y/N): ").lower() == 'y':
    #     save_samples(random_samples, 'random_samples.csv')
    # if input("Save grid samples? (y/N): ").lower() == 'y':
    #     save_samples(grid_samples, 'grid_samples.csv')
    # if input("Save halton samples? (y/N): ").lower() == 'y':
    #     save_samples(halton_samples, 'halton_samples.csv')

    return random_samples, grid_samples, halton_samples

def visualize_roadmap(samples, roadmap, color):
    fig, ax = plt.subplots()
    
    # Plot the samples
    ax.scatter(samples[:, 0], samples[:, 1], color=color, s=10)
    
    # Plot the edges
    for edge in roadmap:
        point1, point2 = samples[edge[0]], samples[edge[1]]
        ax.plot([point1[0], point2[0]], [point1[1], point2[1]], color=color, lw=0.5)
    
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([])
    ax.set_yticks([])
    
    plt.show()

def main():
    # Call the visualize sampling function
    _, grid_samples, _ = visualize_sampling()

    # Random roadmap construction
    random_samples = load_samples('random_samples.csv')
    # random_distance_roadmap = construct_distance_roadmap(random_samples, 0.2)
    # random_degree_roadmap = construct_degree_roadmap(random_samples, 5)
    random_distance_roadmap = load_roadmap('random_distance_roadmap.csv')
    random_degree_roadmap = load_roadmap('random_degree_roadmap.csv')
    visualize_roadmap(random_samples, random_distance_roadmap, 'red')
    visualize_roadmap(random_samples, random_degree_roadmap, 'red')
    # Save roadmap to file
    # if input("Save random distance roadmap? (y/N): ").lower() == 'y':
    #     save_roadmap(random_distance_roadmap, 'random_distance_roadmap.csv')
    # if input("Save random degree roadmap? (y/N): ").lower() == 'y':
    #     save_roadmap(random_degree_roadmap, 'random_degree_roadmap.csv')

    # Grid roadmap construction
    grid_samples = load_samples('grid_samples.csv')
    grid_distance_roadmap = construct_distance_roadmap(grid_samples, 0.2)
    grid_degree_roadmap = construct_degree_roadmap(grid_samples, 5)
    # grid_distance_roadmap = load_roadmap('grid_distance_roadmap.csv')
    # grid_degree_roadmap = load_roadmap('grid_degree_roadmap.csv')
    visualize_roadmap(grid_samples, grid_distance_roadmap, 'blue')
    visualize_roadmap(grid_samples, grid_degree_roadmap, 'blue')
    # Save roadmap to file
    # if input("Save grid distance roadmap? (y/N): ").lower() == 'y':
    #     save_roadmap(grid_distance_roadmap, 'grid_distance_roadmap.csv')
    # if input("Save grid degree roadmap? (y/N): ").lower() == 'y':
    #     save_roadmap(grid_degree_roadmap, 'grid_degree_roadmap.csv')

    # Halton roadmap construction
    halton_samples = load_samples('halton_samples.csv')
    # halton_distance_roadmap = construct_distance_roadmap(halton_samples, 0.2)
    # halton_degree_roadmap = construct_degree_roadmap(halton_samples, 5)
    halton_distance_roadmap = load_roadmap('halton_distance_roadmap.csv')
    halton_degree_roadmap = load_roadmap('halton_degree_roadmap.csv')
    visualize_roadmap(halton_samples, halton_distance_roadmap, 'green')
    visualize_roadmap(halton_samples, halton_degree_roadmap, 'green')
    # Save roadmap to file
    # if input("Save halton distance roadmap? (y/N): ").lower() == 'y':
    #     save_roadmap(halton_distance_roadmap, 'halton_distance_roadmap.csv')
    # if input("Save halton degree roadmap? (y/N): ").lower() == 'y':
    #     save_roadmap(halton_degree_roadmap, 'halton_degree_roadmap.csv')

if __name__ == '__main__':
    main()