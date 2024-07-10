import os
import unittest
import pybullet as p

from robots.robot import Robot
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
            input("Press Enter to close the simulation...")
        p.disconnect(cls.physics_client)

    def setUp(self):
        p.resetSimulation()
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())

    def test_robot_initialization(self):
        urdf_path = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf/panda_arm.urdf")
        robot = Robot(urdf_path)
        self.assertTrue(robot.robot_id >= 0, "Failed to initialize Robot")
        # Print all attributes of the robot
        print("\nRobot attributes:")
        print(f"\nURDF Path: {robot.urdf_path}")
        print(f"\nFixed Base: {robot.fixed_base}")
        print(f"\nBase Position: {robot.base_position}")
        print(f"\nBase Orientation: {robot.base_orientation}")
        print(f"\nScale: {robot.scale}")
        print(f"\nRobot ID: {robot.robot_id}")
        print(f"\nJoints: {robot.joints}")
        print(f"\nConfig Space: {robot.config_space}")
        print(f"\nDimension: {robot.dimension}")

    # def test_robot_get_set_pose(self):
    #     urdf_path = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf/panda_arm.urdf")
    #     robot = Robot(urdf_path)
    #     central_pose = (0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0)
    #     minimum_pose = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
    #     maximum_pose = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
    #     random_pose = (-1.434281929137999, 0.27757619690247505, -2.3276861697284536, -1.4734199503841374, 1.5319987163381463, 2.891383798905186, -0.2025190088121137)
    #     set_pose = central_pose
    #     print(f"\Set Pose: {set_pose}")
    #     robot.set_pose(robot.joints, set_pose)
    #     get_pose = robot.get_pose()
    #     print(f"\Get Pose: {get_pose}")
    #     self.assertEqual(get_pose, set_pose, "Set pose and get pose do not match")

    # def test_robot_execute_motion(self):
    #     urdf_path = os.path.join(os.path.dirname(__file__), "../../models/franka_panda/urdf/panda_arm.urdf")
    #     robot = Robot(urdf_path)
    #     path = [[0] * robot.dimension, [0.1] * robot.dimension, [0.2] * robot.dimension]
    #     robot.execute_motion(path)
    #     final_pose = robot.get_pose()
    #     self.assertEqual(final_pose, path[-1], "Final pose does not match the last position in path")
    
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
        print("1 - Test Robot initialization")
        print("2 - Test Robot get/set pose")
        print("3 - Test Robot execute motion")
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
            suite.addTest(TestRobot('test_robot_execute_motion'))
            runner = unittest.TextTestRunner()
            runner.run(suite)
        elif choice == '0':
            print("Exiting...")
            break
        else:
            print("Invalid choice. Please enter 1, 2, 3 or 0.")

if __name__ == "__main__":
    main()
