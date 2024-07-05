from collections import namedtuple
import colorsys
import numpy as np

"""
Color Utilities

This module provides utility functions and definitions for handling colors within simulations or visualizations.
It includes definitions for basic color tuples, predefined color values, and functions to manipulate and convert
colors for various purposes.
"""

# ---------------------
# Color data structures
RGB = namedtuple('RGB', ['red', 'green', 'blue'])
RGBA = namedtuple('RGBA', ['red', 'green', 'blue', 'alpha'])
MAX_RGB = 2**8 - 1
# ---------------------

# ---------------------
# Colors
RED = RGBA(1, 0, 0, 1)
GREEN = RGBA(0, 1, 0, 1)
BLUE = RGBA(0, 0, 1, 1)
BLACK = RGBA(0, 0, 0, 1)
WHITE = RGBA(1, 1, 1, 1)
BROWN = RGBA(0.396, 0.263, 0.129, 1)
TAN = RGBA(0.824, 0.706, 0.549, 1)
GREY = RGBA(0.5, 0.5, 0.5, 1)
YELLOW = RGBA(1, 1, 0, 1)
TRANSPARENT = RGBA(0, 0, 0, 0)
# ---------------------

# ---------------------
# Dictionaries categorizing colors
ACHROMATIC_COLORS = {
    'white': WHITE,
    'grey': GREY,
    'black': BLACK,
}

CHROMATIC_COLORS = {
    'red': RED,
    'green': GREEN,
    'blue': BLUE,
}
# ---------------------

# ---------------------
# Functions for color manipulation and conversion
def to_8_bit(color):
    '''(0 to 1) to (0 to 255)'''
    return (MAX_RGB*np.array(color)).astype(int).tolist()

def from_8_bit(color):
    '''(0 to 255) to (0 to 1)'''
    return (np.array(color).astype(float) / MAX_RGB).tolist()

def remove_alpha(color):
    return RGB(*color[:3])

def apply_alpha(color, alpha=1.):
    if color is None:
        return None
    red, green, blue = color[:3]
    return RGBA(red, green, blue, alpha)

def spaced_colors(n, s=1, v=1):
    """
    Generates a list of `n` colors spaced evenly around the color wheel, using HSV to RGB conversion.
    
    Parameters:
        n (int): Number of colors to generate.
        s (float): Saturation of the generated colors (0 to 1).
        v (float): Value/Brightness of the generated colors (0 to 1).
        
    Returns:
        A list of RGB namedtuples representing the generated colors.
    """
    return [RGB(*colorsys.hsv_to_rgb(h, s, v)) for h in np.linspace(0, 1, n, endpoint=False)]   # * unpacks an iterable into the arguments
# ---------------------
