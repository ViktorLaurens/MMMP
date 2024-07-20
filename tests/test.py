from collections import deque
import os
import random

import numpy as np

from utils.planner_utils import Interval
if False: 
    new_pose = input("Set arm pose to (or 0 to exit): ")
    new_pose = tuple([float(x) for x in new_pose.split()])
    if not new_pose:
        print("Exiting...")
    print(f"\nSet Pose: {new_pose}")

if False: 
    deq = deque([1, 2, 3, 4, 5])
    print(deq)

if False:
    c_space = [Interval(1, 2), Interval(3, 4), Interval(5, 6)]
    q = np.random.uniform(low=[i.lower for i in c_space], high=[i.upper for i in c_space])
    print(q)

if True:
    print(os.path.abspath('tests/test.py'))
    print(os.path.dirname(os.path.abspath('tests/test.py')))
    print(os.path.dirname(__file__))