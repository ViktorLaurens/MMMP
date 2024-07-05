import os
import unittest
import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt

class TestURDFLoading(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.physics_client = p.connect(p.DIRECT) # p.DIRECT or p.GUI

    @classmethod
    def tearDownClass(cls):
        p.disconnect(cls.physics_client)

    def setUp(self):
        p.resetSimulation()

    def test_load_urdf(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf")
        self.assertTrue(os.path.exists(urdf_dir), "URDF directory does not exist")
        urdf_file = os.path.join(urdf_dir, 'panda_arm_hand.urdf')
        self.assertTrue(os.path.isfile(urdf_file), "URDF file does not exist")
        robot_id = p.loadURDF(urdf_file)
        self.assertGreaterEqual(robot_id, 0, "Failed to load URDF file")

    def test_load_urdf_execution_time(self):
        urdf_dir = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf")
        urdf_file = os.path.join(urdf_dir, 'panda_arm_hand.urdf')
        start_time = time.time()
        p.loadURDF(urdf_file)
        execution_time = time.time() - start_time
        print(f"URDF loading execution time: {execution_time:.6f} seconds")
        return execution_time

def plot_execution_times(execution_times):
    plt.figure()
    plt.plot(execution_times, marker='o')
    plt.title('URDF Loading Execution Times')
    plt.xlabel('Run')
    plt.ylabel('Time (seconds)')
    plt.show()

def main():
    while True: 
        print("\nSelect an action:")
        print("1 - Test URDF Loading")
        print("2 - Test URDF Loading Execution Time")
        print("3 - Plot URDF Loading Execution Times")
        print("0 - Exit")
        choice = input("Enter your choice: ")

        if choice == '1':
            suite = unittest.TestSuite()
            suite.addTest(TestURDFLoading('test_load_urdf'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '2':
            suite = unittest.TestSuite()
            suite.addTest(TestURDFLoading('test_load_urdf_execution_time'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '3':
            execution_times = []
            count = int(input('Enter the number of times to run the test:'))
            for _ in range(count):  # Run the execution time test 'count' times
                test_instance = TestURDFLoading()
                test_instance.setUpClass()
                execution_times.append(test_instance.test_load_urdf_execution_time())
                test_instance.tearDownClass()
            plot_execution_times(execution_times)
        elif choice == '0':
            print("Exiting...")
            break
        else:
            print("Invalid choice. Please enter 1, 2, 3 or 0.")

if __name__ == "__main__":
    main()
