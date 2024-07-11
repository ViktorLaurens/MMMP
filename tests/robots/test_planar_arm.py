import unittest
from matplotlib import pyplot as plt
import numpy as np
from robots.planar_arm import PlanarArm
from utils.planner_utils import Interval

class TestPlanarArm(unittest.TestCase):
    def test_initialization(self):
        links = np.array([1.0, 1.0])
        arm = PlanarArm(links)
        self.assertEqual(arm.n_links, 2)
        self.assertTrue(np.array_equal(arm.base_pos, np.array([0, 0])))
        self.assertTrue(np.array_equal(arm.q, np.zeros(2)))
        self.assertTrue(np.array_equal(arm.q_d, np.zeros(2)))
        self.assertTrue(np.array_equal(arm.pos, np.zeros((2, 2))))
        self.assertEqual(arm.config_space, [Interval(lower=-np.pi, upper=np.pi) for _ in range(2)])
        print(f"Initialization: {arm.__dict__}")

    def test_forward_kinematics(self):
        links = np.array([1.0, 1.0])
        arm = PlanarArm(links)
        q = np.array([np.pi/2, np.pi/2])
        pos = arm.forward_kinematics(q)
        expected_pos = np.array([[0, 1], [-1, 1]])
        np.testing.assert_almost_equal(pos, expected_pos, decimal=5)
        print(f"Forward Kinematics: {pos}")

    def test_set_pose(self):
        links = np.array([1.0, 1.0])
        arm = PlanarArm(links)
        q = np.array([np.pi/2, np.pi/2])
        arm.set_pose(q)
        np.testing.assert_almost_equal(arm.q, q, decimal=5)
        expected_pos = np.array([[0, 1], [-1, 1]])
        np.testing.assert_almost_equal(arm.pos, expected_pos, decimal=5)
        print(f"Set Pose: q={arm.q}, pos={arm.pos}")

    def test_distance(self):
        links = np.array([1.0, 1.0])
        arm = PlanarArm(links)
        pos1 = np.array([[0, 1], [-1, 1]])
        pos2 = np.array([[1, 0], [1, -1]])
        distance = arm.distance(pos1, pos2)
        print(f"Distance: {distance}")
    
def main():
    while True:
        print("\nSelect an action:")
        print("1 - Test PlanarArm initialization")
        print("2 - Test PlanarArm forward kinematics")
        print("3 - Test PlanarArm set pose")
        print("4 - Test PlanarArm distance")
        print("0 - Exit")
        choice = input("Enter your choice: ")
        suite = unittest.TestSuite()
        if choice == '1':
            suite.addTest(TestPlanarArm('test_initialization'))
        elif choice == '2':
            suite.addTest(TestPlanarArm('test_forward_kinematics'))
        elif choice == '3':
            suite.addTest(TestPlanarArm('test_set_pose'))
        elif choice == '4':
            suite.addTest(TestPlanarArm('test_distance'))
        elif choice == '0':
            print("Exiting...")
            break
        else:
            print("Invalid choice. Please enter a valid number.")
            continue

        runner = unittest.TextTestRunner()
        runner.run(suite)

if __name__ == '__main__':
    main()

