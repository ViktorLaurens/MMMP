"""
Data handling

This module contains functions for processing or modifying data.
"""

import os
import re


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