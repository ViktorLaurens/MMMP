import os
import random
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
        panda = Panda(base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        self.assertTrue(panda.robot_id >= 0, "Failed to initialize Robot")
        # Print all attributes of the robot
        print("\nRobot attributes:")
        print(f"\n - URDF Path: {panda.urdf_path}")
        print(f"\n - Fixed Base: {panda.fixed_base}")
        print(f"\n - Base Position: {panda.base_position}")
        print(f"\n - Base Orientation: {panda.base_orientation}")
        print(f"\n - Scale: {panda.scale}")
        print(f"\n - Robot ID: {panda.robot_id}")
        print(f"\n - Joints: {panda.joints}")
        print(f"\n - Arm joints: {panda.arm_joints}")
        print(f"\n - Gripper joints: {panda.gripper_joints}")
        print(f"\n - C-Space: {panda.c_space}")
        print(f"\n - Arm C-Space: {panda.arm_c_space}")
        print(f"\n - Gripper C-Space: {panda.gripper_c_space}")
        print(f"\n - Dimension: {panda.dimension}")
        print(f"\n - Arm Dimension: {panda.arm_dimension}")
        print(f"\n - Gripper Dimension: {panda.gripper_dimension}")
    
    def test_get_arm_pose(self):
        panda = Panda(base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        random_pose = generate_random_pose(panda.arm_c_space)
        panda.set_arm_pose(random_pose)
        get_pose = panda.get_arm_pose()
        print(f"\nRandom Pose: {random_pose}")
        print(f"\nGet Arm Pose: {get_pose}")
        self.assertEqual(get_pose, random_pose, "Random pose and get arm pose do not match")

    def test_get_gripper_pose(self):
        panda = Panda(base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        random_pose = generate_random_pose(panda.gripper_c_space)
        panda.set_gripper_pose(random_pose)
        get_pose = panda.get_gripper_pose()
        print(f"\nRandom Pose: {random_pose}")
        print(f"\nGet Gripper Pose: {get_pose}")
        self.assertEqual(get_pose, random_pose, "Random pose and get gripper pose do not match")

    def test_set_arm_pose(self):
        panda = Panda(base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        while True:
            new_pose = input("Set arm pose to: ")
            try: 
                new_pose = tuple([float(x) for x in new_pose.split()])
            except ValueError:
                ValueError("Invalid input. Please enter a series of floats separated by spaces.")

            if not new_pose:
                break
            elif len(new_pose) != panda.arm_dimension:
                ValueError(f"Invalid input. Please enter a tuple of {panda.arm_dimension} floats.") 
            else:
                panda.set_arm_pose(new_pose)
                print(f"Pose set to {new_pose}")
    
    def test_set_gripper_pose(self):
        panda = Panda(base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        while True:
            new_pose = input("Set gripper pose to: ")
            try: 
                new_pose = tuple([float(x) for x in new_pose.split()])
            except ValueError:
                ValueError("Invalid input. Please enter a series of floats separated by spaces.")
            
            if not new_pose:
                break
            elif len(new_pose) != panda.gripper_dimension:
                ValueError(f"Invalid input. Please enter a tuple of {panda.gripper_dimension} floats.") 
            else:
                panda.set_gripper_pose(new_pose)
                print(f"Pose set to {new_pose}")

    def test_set_arm_in_standby(self):
        panda = Panda(base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        panda.set_arm_in_standby()
        get_pose = panda.get_arm_pose()
        print(f"\nGet Arm Pose: {get_pose}")
        self.assertEqual(get_pose, (0, 0, 0, 0, 0, 0, 0), "Arm pose is not in standby position")
    
    def test_set_arm_in_neutral(self):
        panda = Panda(base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        panda.set_arm_in_neutral()
        get_pose = panda.get_arm_pose()
        print(f"\nGet Arm Pose: {get_pose}")
        self.assertEqual(get_pose, (0, 0, 0, -1.51, 0, 1.877, 0), "Arm pose is not in neutral position")

    def test_set_gripper_open(self):
        panda = Panda(base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        panda.set_gripper_open()
        get_pose = panda.get_gripper_pose()
        print(f"\nGet Gripper Pose: {get_pose}")
        self.assertEqual(get_pose, (0.04, 0.04), "Gripper is not open")

    def test_set_gripper_close(self):
        panda = Panda(base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        panda.set_gripper_close()
        get_pose = panda.get_gripper_pose()
        print(f"\nGet Gripper Pose: {get_pose}")
        self.assertEqual(get_pose, (0, 0), "Gripper is not closed")

    def test_execute_arm_motion(self):
        panda = Panda(fixed_base=True, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        initial_pose = (0.0, 0.0, -2.0, -1.5708, 0.0, 1.5708, 0.0)
        goal_pose = (0.0, 0.0, 2.0, -1.5708, 0.0, 1.5708, 0.0)
        nr_frames = 120
        path = calculate_straight_path(initial_pose, goal_pose, nr_frames)
        panda.execute_arm_motion(path)
        final_pose = panda.get_pose()
        self.assertEqual(final_pose, path[-1], "Final pose does not match the last position in path")
    
    def test_execute_gripper_motion(self): 
        panda = Panda(fixed_base=True, base_position=(0, 0, 0), base_orientation=(0, 0, 0, 1))
        open_pose = (0.04, 0.04)
        close_pose = (0, 0)
        nr_frames = 30  
        path = calculate_straight_path(open_pose, close_pose, nr_frames)
        panda.execute_gripper_motion(path)
        final_pose = panda.get_gripper_pose()
        self.assertEqual(final_pose, path[-1], "Final pose does not match the last position in path")
        
def generate_random_pose(c_space):
    return tuple([random.uniform(c_space[i][0], c_space[i][1]) for i in range(len(c_space))])

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
        print("1 - Test panda initialization")
        print("2 - Test get arm pose")
        print("3 - Test get gripper pose")
        print("4 - Test set arm pose")
        print("5 - Test set gripper pose")
        print("6 - Test set arm in standby")
        print("7 - Test set arm in neutral")
        print("8 - Test set gripper open")
        print("9 - Test set gripper close")
        print("10 - Test execute arm motion")
        print("11 - Test execute gripper motion")
        print("0 - Exit")
        choice = input("Enter your choice: ")
        if choice == '1':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_robot_initialization'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '2':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_get_arm_pose'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '3':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_get_gripper_pose'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '4':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_set_arm_pose'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '5':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_set_gripper_pose'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '6':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_set_arm_in_standby'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '7':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_set_arm_in_neutral'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '8':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_set_gripper_open'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '9':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_set_gripper_close'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '10':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_execute_arm_motion'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '11':
            suite = unittest.TestSuite()
            suite.addTest(TestRobot('test_execute_gripper_motion'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '0':
            print("Exiting...\n")
            break
        else:
            print("Invalid choice. Please enter 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 or 0.")

if __name__ == "__main__":
    main()
