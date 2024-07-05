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
    def test_load_urdf_hand(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf")
        self.assertTrue(os.path.exists(urdf_dir), "URDF directory does not exist")
        urdf_file = os.path.join(urdf_dir, 'hand.urdf')
        self.assertTrue(os.path.isfile(urdf_file), "URDF file does not exist")
        robot_id = p.loadURDF(urdf_file)
        self.assertGreaterEqual(robot_id, 0, "Failed to load URDF file")

    def test_load_urdf_arm(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf")
        self.assertTrue(os.path.exists(urdf_dir), "URDF directory does not exist")
        urdf_file = os.path.join(urdf_dir, 'panda_arm.urdf')
        self.assertTrue(os.path.isfile(urdf_file), "URDF file does not exist")
        robot_id = p.loadURDF(urdf_file)
        self.assertGreaterEqual(robot_id, 0, "Failed to load URDF file")

    def test_load_urdf_arm_hand(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf")
        self.assertTrue(os.path.exists(urdf_dir), "URDF directory does not exist")
        urdf_file = os.path.join(urdf_dir, 'panda_arm_hand.urdf')
        self.assertTrue(os.path.isfile(urdf_file), "URDF file does not exist")
        robot_id = p.loadURDF(urdf_file)
        self.assertGreaterEqual(robot_id, 0, "Failed to load URDF file")

    # Test the execution time of loading the URDF file
    def test_load_urdf_hand_execution_time(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf")
        urdf_file = os.path.join(urdf_dir, 'hand.urdf')
        start_time = time.time()
        p.loadURDF(urdf_file)
        execution_time = time.time() - start_time
        print(f"URDF loading execution time: {execution_time:.6f} seconds")
        return execution_time

    def test_load_urdf_arm_execution_time(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf")
        urdf_file = os.path.join(urdf_dir, 'panda_arm.urdf')
        start_time = time.time()
        p.loadURDF(urdf_file)
        execution_time = time.time() - start_time
        print(f"URDF loading execution time: {execution_time:.6f} seconds")
        return execution_time

    def test_load_urdf_arm_hand_execution_time(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf")
        urdf_file = os.path.join(urdf_dir, 'panda_arm_hand.urdf')
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
        print("\nSelect an action:")
        print("1 - Test hand")
        print("2 - Test arm")
        print("3 - Plot arm + hand")
        print("0 - Exit")
        choice = input("Enter your choice: ")

        if choice == '1':
            while True: 
                print("\nSelect an action:")
                print("1 - Test URDF loading")
                print("2 - Test loading execution time")
                print("3 - Plot loading execution times")
                print("0 - Exit")
                choice2 = input("Enter your choice: ")
                if choice2 == '1':
                    suite = unittest.TestSuite()
                    suite.addTest(TestURDFLoading('test_load_urdf_hand'))
                    runner = unittest.TextTestRunner()
                    runner.run(suite)
                elif choice2 == '2':
                    suite = unittest.TestSuite()
                    suite.addTest(TestURDFLoading('test_load_urdf_hand_execution_time'))
                    runner = unittest.TextTestRunner()
                    runner.run(suite)
                elif choice2 == '3':
                    execution_times = []
                    count = int(input('Enter the number of times to run the test:'))
                    for _ in range(count):  # Run the execution time test 'count' times
                        test_instance = TestURDFLoading()
                        test_instance.setUpClass()
                        execution_times.append(test_instance.test_load_urdf_hand_execution_time())
                        test_instance.tearDownClass()   
                    plot_execution_times(execution_times)
                elif choice2 == '0':
                    print("Exiting...")
                    break
                else:
                    print("Invalid choice. Please enter 1, 2, 3 or 0.")

        elif choice == '2':
            while True: 
                print("\nSelect an action:")
                print("1 - Test URDF loading")
                print("2 - Test loading execution time")
                print("3 - Plot loading execution times")
                print("0 - Exit")
                choice2 = input("Enter your choice: ")
                if choice2 == '1':
                    suite = unittest.TestSuite()
                    suite.addTest(TestURDFLoading('test_load_urdf_arm'))
                    runner = unittest.TextTestRunner()
                    runner.run(suite)
                elif choice2 == '2':
                    suite = unittest.TestSuite()
                    suite.addTest(TestURDFLoading('test_load_urdf_arm_execution_time'))
                    runner = unittest.TextTestRunner()
                    runner.run(suite)
                elif choice2 == '3':
                    execution_times = []
                    count = int(input('Enter the number of times to run the test:'))
                    for _ in range(count):  # Run the execution time test 'count' times
                        test_instance = TestURDFLoading()
                        test_instance.setUpClass()
                        execution_times.append(test_instance.test_load_urdf_arm_execution_time())
                        test_instance.tearDownClass()   
                    plot_execution_times(execution_times)
                elif choice2 == '0':
                    print("Exiting...")
                    break
                else:
                    print("Invalid choice. Please enter 1, 2, 3 or 0.")

        elif choice == '3':
            while True: 
                print("\nSelect an action:")
                print("1 - Test URDF loading")
                print("2 - Test loading execution time")
                print("3 - Plot loading execution times")
                print("0 - Exit")
                choice2 = input("Enter your choice: ")
                if choice2 == '1':
                    suite = unittest.TestSuite()
                    suite.addTest(TestURDFLoading('test_load_urdf_arm_hand'))
                    runner = unittest.TextTestRunner()
                    runner.run(suite)
                elif choice2 == '2':
                    suite = unittest.TestSuite()
                    suite.addTest(TestURDFLoading('test_load_urdf_arm_hand_execution_time'))
                    runner = unittest.TextTestRunner()
                    runner.run(suite)
                elif choice2 == '3':
                    execution_times = []
                    count = int(input('Enter the number of times to run the test:'))
                    for _ in range(count):  # Run the execution time test 'count' times
                        test_instance = TestURDFLoading()
                        test_instance.setUpClass()
                        execution_times.append(test_instance.test_load_urdf_arm_hand_execution_time())
                        test_instance.tearDownClass()   
                    plot_execution_times(execution_times)
                elif choice2 == '0':
                    print("Exiting...")
                    break
                else:
                    print("Invalid choice. Please enter 1, 2, 3 or 0.")

        elif choice == '0':
            print("Exiting...")
            break

        else:
            print("Invalid choice. Please enter 1, 2, 3 or 0.")

if __name__ == "__main__":
    main()
