"""
Data processing

This module contains functions for processing or modifying data.
"""

def safe_zip(sequence1, sequence2):
    sequence1, sequence2 = list(sequence1), list(sequence2)
    assert len(sequence1) == len(sequence2)
    return list(zip(sequence1, sequence2))