import math
import os
import unittest
import numpy as np
import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt

from utils.pybullet_utils import set_camera_pose

class TestURDFLoading(unittest.TestCase):
    model = 'obstacle'  # Model name
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
        if cls.connection_type == p.DIRECT:
            p.disconnect(cls.physics_client)
        else:
            input("Press Enter to disconnect and close the PyBullet GUI...")
            p.disconnect(cls.physics_client)

    def setUp(self):
        p.resetSimulation()

    # Test loading the URDF file
    def test_load_urdf(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/obstacles")
        self.assertTrue(os.path.exists(urdf_dir), "URDF directory does not exist")
        urdf_file = os.path.join(urdf_dir, self.model + '.urdf')
        self.assertTrue(os.path.isfile(urdf_file), "URDF file does not exist")
        robot_id = p.loadURDF(urdf_file)
        self.assertGreaterEqual(robot_id, 0, "Failed to load URDF file")

    # Test the execution time of loading the URDF file
    def test_load_urdf_execution_time(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/obstacles")
        urdf_file = os.path.join(urdf_dir, self.model + '.urdf')
        start_time = time.time()
        p.loadURDF(urdf_file)
        execution_time = time.time() - start_time
        print(f"URDF loading execution time: {execution_time:.6f} seconds")
        return execution_time

def plot_execution_times(execution_times):
    average_time = np.mean(execution_times)
    plt.figure()
    plt.plot(execution_times, marker='o', label=f'Execution Times (avg: {average_time:.6f} s)')
    plt.title('URDF Loading Execution Times')
    plt.xlabel('Run')
    plt.ylabel('Time (seconds)')
    plt.legend()
    plt.show()
    print(f'Average execution time: {average_time:.6f} seconds')

def main():
    connection_type = None
    while connection_type not in ['DIRECT', 'GUI']:
        print("\nSelect PyBullet connection type:")
        print("1 - DIRECT")
        print("2 - GUI")
        choice = input("Enter your choice (1 or 2): ")
        if choice == '1':
            connection_type = 'DIRECT'
            TestURDFLoading.connection_type = p.DIRECT
        elif choice == '2':
            connection_type = 'GUI'
            TestURDFLoading.connection_type = p.GUI
        else:
            print("Invalid choice. Please enter 1 or 2.")

    while True: 
        print("\nSelect an obstacle to test:")
        print("1 - big_robot_toy")
        print("2 - black_box")
        print("3 - block_for_pick_and_place_large_size")
        print("4 - block_for_pick_and_place_mid_size")
        print("5 - block_for_pick_and_place")
        print("6 - folding_table")
        print("7 - round_table")
        print("8 - simple_cuboid")
        print("9 - simple_cylinder")
        print("10 - yellow_post")
        print("11 - cup_small")
        print("0 - Exit")
        choice = input("Enter your choice: ")

        if choice == '1':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'big_robot_toy'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '2':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'black_box'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '3':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'block_for_pick_and_place_large_size'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '4':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'block_for_pick_and_place_mid_size'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '5':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'block_for_pick_and_place'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '6':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'folding_table'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '7':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'round_table'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '8':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'simple_cuboid'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '9':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'simple_cylinder'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '10':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'yellow_post'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '11':
            suite = unittest.TestSuite()
            TestURDFLoading.model = 'cup_small'
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)

        elif choice == '0':
            print("Exiting...")
            break

        else:
            print("Invalid choice. Please enter 1, 2, 3 or 0.")

if __name__ == "__main__":
    main()
