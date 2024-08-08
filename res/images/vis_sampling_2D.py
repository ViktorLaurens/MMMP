import numpy as np
import matplotlib.pyplot as plt
from scipy.stats.qmc import Halton

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

def save_samples(samples, filename):
    directory = 'res/images/'
    np.savetxt(directory + filename, samples, delimiter=',')
    print(f"Samples saved to {filename}")
    return

def load_samples(filename):
    return np.loadtxt(filename, delimiter=',')


def visualize():
    fig, ax = plt.subplots()
    
    # Number of nodes to plot per method
    n = 50
    point_size = 10  # Size of the points

    # Uniform random sampling
    random_samples = generate_random_samples(n)
    ax.scatter(random_samples[:, 0], random_samples[:, 1], color='red', s=point_size, label='Uniform Random')

    # Grid sampling with Gaussian noise
    grid_samples = generate_grid_samples(n)
    ax.scatter(grid_samples[:, 0], grid_samples[:, 1], color='blue', s=point_size, label='Grid with Gaussian Noise')

    # Halton sequence sampling (2, 3 dimensions)
    halton_samples = generate_halton_samples(n)
    ax.scatter(halton_samples[:, 0], halton_samples[:, 1], color='green', s=point_size, label='Halton Sequence (2, 3)')

    # Plot settings
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.legend(loc='upper right')
    
    plt.show()

    # Save samples to file
    if input("Save random samples? (y/N): ").lower() == 'y':
        save_samples(random_samples, 'random_samples.csv')
    if input("Save grid samples? (y/N): ").lower() == 'y':
        save_samples(grid_samples, 'grid_samples.csv')
    if input("Save halton samples? (y/N): ").lower() == 'y':
        save_samples(halton_samples, 'halton_samples.csv')
def main():
    # Call the visualize function
    visualize()

if __name__ == '__main__':
    main()