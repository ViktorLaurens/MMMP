import pybullet as p
import time
from collections import namedtuple
import math 
import numpy as np 

# ---------------------
# VARIABLES 
CLIENT = 0  # Default client ID
RENDERING_ENABLED = True    # Initialize a global variable to keep track of the rendering state

ModelInfo = namedtuple('URDFInfo', ['name', 'path', 'fixed_base', 'scale'])  # Namedtuple for URDF model information
INFO_FROM_BODY = {}  # Dictionary to map body IDs to their corresponding ModelInfo

MAX_DISTANCE = 0. # 0. | 1e-3
# ---------------------

# ---------------------
# CONNECT 
def connect(use_gui=True, shadows=True, color=None, width=None, height=None, mp4=None, fps=120):
    """
    Establishes a connection to the PyBullet simulation environment with customizable options.
    """
    global CLIENT  # Declare CLIENT as global to modify it
    method = p.GUI if use_gui else p.DIRECT
    options = []
    if mp4 is not None:
        options.append('--mp4="{}" --fps={}'.format(mp4, fps))
    if color is not None:
        options.append('--background_color_red={} --background_color_green={} --background_color_blue={}'.format(*color))
    if width is not None:
        options.append(' --width={}'.format(width))
    if height is not None:
        options.append(' --height={}'.format(height))
    CLIENT = p.connect(method, options=' '.join(options))  # Directly store the client ID in CLIENT
    assert CLIENT >= 0, 'Invalid CLIENT, connection unsuccessful'
    if use_gui:
        disable_preview()  # Custom function to disable any default previews
        # Configure the debug visualizer to not use the TinyRenderer (CPU-based renderer) unless necessary
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, False, physicsClientId=CLIENT)
        # Enable or disable shadows in the simulation
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, shadows, physicsClientId=CLIENT)
        # Disable mouse picking to prevent users from moving objects by clicking on them
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, False, physicsClientId=CLIENT)
        # Disable keyboard shortcuts within the simulation to prevent unintended interactions
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, False, physicsClientId=CLIENT)

def set_preview(enable):
    """
    Configures the visibility of various PyBullet GUI features.
    """
    if CLIENT is not None:  # Check if CLIENT has been set
        # Configures the visibility of the PyBullet GUI. When enabled, the GUI is shown; when disabled, it's hidden.
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, enable, physicsClientId=CLIENT)
        # Controls the visibility of the RGB buffer preview in the GUI. This preview shows the rendered image from the camera view.
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, enable, physicsClientId=CLIENT)
        # Controls the visibility of the depth buffer preview in the GUI. This preview provides depth information from the camera's perspective.
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, enable, physicsClientId=CLIENT)
        # Controls the visibility of the segmentation mark preview in the GUI. This is useful for visualizing different segments or objects within the simulation.
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, enable, physicsClientId=CLIENT)
        # Display objects in wireframe mode
        # p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, True, physicsClientId=CLIENT)

def enable_preview():
    set_preview(enable=True)

def disable_preview():
    set_preview(enable=False)

def disconnect():
    global CLIENT
    if CLIENT is not None:  # Check if CLIENT has been set
        p.disconnect(physicsClientId=CLIENT)
        CLIENT = None  # Reset CLIENT to None after disconnection
# ---------------------

# ---------------------
# ADD DATA PATH
def get_data_path():
    """
    Retrieves the default data path provided by the PyBullet library
    """
    import pybullet_data
    return pybullet_data.getDataPath()

def add_data_path(data_path=None):
    """
    Adds a specified data path to PyBullet's search paths for simulation assets. If no path is provided,
    the default PyBullet data path is used.
    """
    if data_path is None:
        data_path = get_data_path()
    p.setAdditionalSearchPath(data_path)    # This allows PyBullet to locate and load assets from this directory.
    return data_path    # Return the data path that was added for reference or further use.
# ---------------------

# ---------------------
# SET CAMERA POSE
def set_camera_pose(camera_point, target_point=np.zeros(3)):
    delta_point = np.array(target_point) - np.array(camera_point)
    distance = np.linalg.norm(delta_point)
    yaw = get_yaw(delta_point) - np.pi/2
    pitch = get_pitch(delta_point)
    p.resetDebugVisualizerCamera(distance, math.degrees(yaw), math.degrees(pitch),
                                 target_point, physicsClientId=CLIENT)
    
def get_view_matrix(camera_point, target_point=np.zeros(3)):
    """
    Compute the view matrix from specification of camera position and target point.

    Args:
    - camera_point (array-like): Coordinates [x, y, z] of the camera.
    - target_point (array-like, optional): Coordinates [x, y, z] of the target the camera looks at. Defaults to origin [0, 0, 0].
    - client_id (int, optional): The PyBullet client ID. Defaults to 0. Unused in this function, added for API consistency.

    Returns:
    - view_matrix (list): The computed view matrix for the camera.
    """
    # Compute the view matrix using PyBullet's function
    view_matrix = p.computeViewMatrix(cameraEyePosition=camera_point,
                                      cameraTargetPosition=target_point,
                                      cameraUpVector=[0, 0, 1])  # Assuming Z-up coordinate system
    return view_matrix

def get_pitch(point):
    dx, dy, dz = point
    return np.math.atan2(dz, np.sqrt(dx ** 2 + dy ** 2))

def get_yaw(point):
    dx, dy = point[:2]
    return np.math.atan2(dy, dx)
# ---------------------

# ---------------------
# LOCK RENDERER
class LockRenderer:
    """
    Temporarily disables rendering to make operations like adding objects faster.
    """
    def __init__(self, lock=True):
        global RENDERING_ENABLED
        self.lock = lock
        self.original_state = RENDERING_ENABLED

    def __enter__(self):
        # Only proceed if GUI is active and locking is requested
        if gui_enabled() and self.lock:
            self.original_state = get_renderer_state()  # Save the current state before changing
            set_renderer_state(enable=False)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Restore the original state if it was modified
        if self.original_state is not None:
            set_renderer_state(enable=self.original_state)

def get_connection_method():
    return p.getConnectionInfo(physicsClientId=CLIENT)['connectionMethod']  # returns 1 if p.GUI, returns 2 if p.DIRECT

def gui_enabled():
    return get_connection_method() == p.GUI

def get_renderer_state():
    """
    Returns whether rendering is currently enabled.
    """
    return RENDERING_ENABLED

def set_renderer_state(enable):
    """
    Enables or disables rendering.
    """
    global RENDERING_ENABLED
    if gui_enabled():
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, int(enable), physicsClientId=CLIENT)
        RENDERING_ENABLED = enable
# ---------------------

# ---------------------
# Prompt message
def pause_sim(prompt='Press enter to continue...'):
    """
    Prompts the user for input to pause execution.
    """
    return input(prompt)
# ---------------------

# ---------------------
# wait for duration
MouseEvent = namedtuple('MouseEvent', ['eventType', 'mousePosX', 'mousePosY', 'buttonIndex', 'buttonState'])

def get_mouse_events():
    return list(MouseEvent(*event) for event in p.getMouseEvents(physicsClientId=CLIENT))

def update_viewer():
    get_mouse_events()

def wait_for_duration(duration): #, dt=0):
    start_time = time.time()
    while time.time() - start_time <= duration:
        update_viewer()
        # time.sleep(duration/10)
# ---------------------