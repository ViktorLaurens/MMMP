import os
import unittest
import pybullet as p

from robots.panda import Panda
from utils.pb_conf_utils import set_camera_pose 

class TestRobot(unittest.TestCase):
    connection_type = p.DIRECT  # p.DIRECT or p.GUI

    @classmethod
    def setUpClass(cls):
        cls.physics_client = p.connect(cls.connection_type) 
        if cls.connection_type == p.GUI:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)     # Disable the GUI
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1) # Enable shadows
            set_camera_pose(camera_point=[0, -1.2, 1.2])        # Set camera pose

    @classmethod
    def tearDownClass(cls):
        if cls.connection_type == p.GUI:
            input("\nPress Enter to close the simulation...")
        p.disconnect(cls.physics_client)

    def setUp(self):
        p.resetSimulation()
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())

    def test_robot_initialization(self):
        panda = Panda(fixed_base=True, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        os.system('cls' if os.name == 'nt' else 'clear') # clear the terminal  
        self.assertTrue(panda.robot_id >= 0, "Failed to initialize Robot")
        # Print all attributes of the robot
        print("\nRobot attributes:")
        print(f" - URDF Path: {panda.urdf_path}")
        print(f" - Fixed Base: {panda.fixed_base}")
        print(f" - Base Position: {panda.base_position}")
        print(f" - Base Orientation: {panda.base_orientation}")
        print(f" - Scale: {panda.scale}")
        print(f" - Robot ID: {panda.robot_id}")
        print(f" - Joints: {panda.joints}")
        print(f" - Arm joints: {panda.arm_joints}")
        print(f" - Gripper joints: {panda.gripper_joints}")
        print(f" - C-Space: {panda.c_space}")
        print(f" - Arm C-Space: {panda.arm_c_space}")
        print(f" - Gripper C-Space: {panda.gripper_c_space}")
        print(f" - Dimension: {panda.dimension}")
        print(f" - Arm Dimension: {panda.arm_dimension}")
        print(f" - Gripper Dimension: {panda.gripper_dimension}")

    def test_robot_get_set_pose(self):
        panda = Panda(fixed_base=True, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        os.system('cls' if os.name == 'nt' else 'clear') # clear the terminal  
        central_pose = (0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0)
        minimum_pose = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
        maximum_pose = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
        random_pose = (-1.434281929137999, 0.27757619690247505, -2.3276861697284536, -1.4734199503841374, 1.5319987163381463, 2.891383798905186, -0.2025190088121137)
        set_pose = central_pose
        print(f"\Set Pose: {set_pose}")
        panda.set_pose(panda.joints, set_pose)
        get_pose = panda.get_pose()
        print(f"\Get Pose: {get_pose}")
        self.assertEqual(get_pose, set_pose, "Set pose and get pose do not match")

    def set_robot_pose(self): 
        panda = Panda(fixed_base=True, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        while True:
            new_pose = input("Set pose to (or 0 to exit): ")
            if new_pose == '0':
                break
            if new_pose.isinstance(tuple):
                ValueError("Invalid input. Please enter a tuple of floats.")
            elif len(new_pose.split()) != panda.dimension:
                ValueError(f"Invalid input. Please enter a tuple of {panda.dimension} floats.") 
            else:
                panda.set_pose(panda.joints, new_pose)
                print(f"Pose set to {new_pose}")

    def test_robot_execute_motion(self):
        panda = Panda(fixed_base=True, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        initial_pose = (0.0, 0.0, -2.0, -1.5708, 0.0, 1.5708, 0.0)
        goal_pose = (0.0, 0.0, 2.0, -1.5708, 0.0, 1.5708, 0.0)
        nr_frames = 10
        path = calculate_straight_path(initial_pose, goal_pose, nr_frames)
        panda.execute_motion(path)
        final_pose = panda.get_pose()
        self.assertEqual(final_pose, path[-1], "Final pose does not match the last position in path")

def calculate_straight_path(start, goal, nr_frames):
    path = []
    for i in range(nr_frames):
        path.append([start[j] + i * (goal[j] - start[j]) / nr_frames for j in range(len(start))])
    return path

def main():
    connection_type = None
    while connection_type not in ['DIRECT', 'GUI']:
        print("\nSelect PyBullet connection type:")
        print("1 - DIRECT")
        print("2 - GUI")
        choice = input("Enter your choice (1 or 2): ")
        if choice == '1':
            connection_type = 'DIRECT'
            TestRobot.connection_type = p.DIRECT
        elif choice == '2':
            connection_type = 'GUI'
            TestRobot.connection_type = p.GUI
        else:
            print("Invalid choice. Please enter 1 or 2.")
    
    while True:
        print("\nSelect an action:")
        print("1 - Test Panda initialization")
        print("2 - Test Panda get/set pose")
        print("3 - Set panda pose")
        print("4 - Test Panda execute motion")
        print("0 - Exit")
        choice = input("Enter your choice: ")
        if choice == '1':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_robot_initialization'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '2':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_robot_get_set_pose'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '3':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('set_robot_pose'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '4':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_robot_execute_motion'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '0':
            print("Exiting...\n")
            break
        else:
            print("Invalid choice. Please enter 1, 2, 3 or 0.")

if __name__ == "__main__":
    main()
