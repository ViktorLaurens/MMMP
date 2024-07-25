import os
import matplotlib.pyplot as plt

def plot_eps_file(file_path):
    # Open and display the .eps file using matplotlib
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.imshow(plt.imread(file_path))
    ax.axis('off')  # Hide axes
    ax.set_title(os.path.basename(file_path))
    plt.show()

if __name__ == '__main__':
    file_path = os.path.join(os.path.dirname(__file__), 'planar', 'exp3', 'N_QT_CBS_PRIO.eps')
    plot_eps_file(file_path)
