"""
Data handling

This module contains functions for processing or modifying data.
"""

import os
import re

import numpy as np


def safe_zip(sequence1, sequence2):
    sequence1, sequence2 = list(sequence1), list(sequence2)
    assert len(sequence1) == len(sequence2)
    return list(zip(sequence1, sequence2))

def load_roadmap(filename):
    roadmap = {}
    
    # Check if file exists
    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        return roadmap

    # Improved regex to match scientific notation, including negative exponents
    float_pattern = r'-?\d+(?:\.\d+)?(?:[eE]-?\d+)?'

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            
            # Split the line into the node and its neighbors
            try: 
                node_part, neighbors_part = line.split(":")
            except ValueError:
                continue
                print(f"Invalid line: {line}")

            # Convert the node part into a tuple of floats and recast within joint limits
            node = tuple(map(float, re.findall(float_pattern, node_part)))
            
            # Split the neighbors part, convert each neighbor into a tuple of floats, and recast
            neighbors = [
                tuple(map(float, re.findall(float_pattern, neighbor)))
                for neighbor in neighbors_part.split('),(')
            ]
            
            # Update the roadmap dictionary
            roadmap[node] = neighbors

    return roadmap

# Video and GIF creation
def save_video(frames, directory, filename, fps=30):
    """
    Saves a list of frames as a video file.

    Args:
    - frames (list): A list of frames to save as a video.
    - directory (str): The directory to save the video file.
    - filename (str): The name of the video file to save.
    - fps (int, optional): The frames per second of the video. Defaults to 30.
    """
    import imageio

    # Ensure frames are in a correct format
    frames = [np.uint8(frame) for frame in frames]  # Convert each frame to uint8 if necessary

    # Save the captured frames as a video
    imageio.mimsave(os.path.join(directory, filename), frames, fps=fps)  # Define the FPS as needed
    print(f"Video saved as {os.path.join(directory, filename)}")

def save_gif(frames, directory, filename, duration=33):
    """
    Saves a list of frames as a GIF file.

    Args:
    - frames (list): A list of frames to save as a GIF.
    - directory (str): The directory to save the GIF file.
    - filename (str): The name of the GIF file to save.
    - duration (int, optional): The duration of each frame in milliseconds. Defaults to 33.
    """
    import imageio

    # Ensure frames are in a correct format
    frames = [np.uint8(frame) for frame in frames]  # Convert each frame to uint8 if necessary

    # Save the captured frames as a GIF
    imageio.mimsave(os.path.join(directory, filename), frames, duration=duration, loop=0)  # Define the duration or fps as needed
    print(f"GIF saved as {os.path.join(directory, filename)}")