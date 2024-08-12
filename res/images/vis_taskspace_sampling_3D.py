import time
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import re

def inverse_kinematics(panda, target_position, target_orientation, default_joint_positions):
    joint_poses = p.calculateInverseKinematics(
        panda, 
        endEffectorLinkIndex=8,  # End effector link index for Panda
        targetPosition=target_position, 
        targetOrientation=target_orientation,
        lowerLimits=[-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159],
        upperLimits=[2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159],
        jointRanges=[5.4874, 3.5674, 5.8014, 2.8903, 5.6130, 3.9724, 6.0318],
        restPoses=default_joint_positions,
        jointDamping=[1e-3, 1e-3, 1e+4, 1e-3, 1e+4, 1e+0, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6], 
        maxNumIterations=2000,
        residualThreshold=1e-4
    )
    return joint_poses

def forward_kinematics(panda, joint_poses):
    p.resetJointStateMultiDof(panda, len(joint_poses), joint_poses)
    link_state = p.getLinkState(panda, 8, computeForwardKinematics=True)
    position = link_state[4]
    orientation = link_state[5]
    return np.array(position), np.array(orientation)

def self_collision(robot_id):
    # Define a small distance threshold for collision detection (0.0 means contact, >0.0 means near contact)
    collision_threshold = 0.0

    num_joints = 7 # dont include grippers since they are allowed to collide to grip
    
    # Check for self-collisions by iterating over all pairs of links
    for i in range(num_joints - 2):
        for j in range(i + 2, num_joints):
            contact_points = p.getClosestPoints(bodyA=robot_id, bodyB=robot_id, linkIndexA=i, linkIndexB=j, distance=collision_threshold)
            
            if contact_points:
                # If there are contact points within the threshold distance, it means there's a collision
                return True

    # No self-collision detected
    return False

def within_joint_limits(q):
    lower_limits = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
    upper_limits = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]
    
    for i, q_i in enumerate(q):
        if q_i < lower_limits[i] or q_i > upper_limits[i]:
            return False
    return True

def plot_task_space_positions(positions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='b', marker='o')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # plt.title('Valid Task-Space Positions for Panda End Effector')
    plt.show()


# Main function to sample task space positions and validate them using IK and FK
def main1():
    # Initialize PyBullet with cleaner settings
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    # Load Panda URDF
    print("Loading Panda URDF...")
    panda_urdf_path = "franka_panda/panda.urdf"
    panda = p.loadURDF(panda_urdf_path, useFixedBase=True)

    # Get the number of joints
    num_joints = p.getNumJoints(panda)
    print(f"Panda has {num_joints} joints.")

    # Define grid resolution and task space bounds
    x_bounds = [-0.855, 0.855]
    y_bounds = [-0.855, 0.855]
    z_bounds = [-0.360, 1.190]
    resolution = 0.1  # Adjust resolution if needed

    # Generate a grid of points in task space
    x_vals = np.arange(x_bounds[0], x_bounds[1] + resolution, resolution)
    y_vals = np.arange(y_bounds[0], y_bounds[1] + resolution, resolution)
    z_vals = np.arange(z_bounds[0], z_bounds[1] + resolution, resolution)
    grid_points = np.array(np.meshgrid(x_vals, y_vals, z_vals)).T.reshape(-1, 3)

    print(f"Generated {len(grid_points)} task-space points.")

    # Define the default pose and orientation
    default_pose = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
    target_orientation = [1, 0, 0, 0]  # End effector pointing downwards

    # Initialize an empty list for valid positions
    valid_positions = []

    # Define a position error margin
    position_error_margin = 0.01

    print("Starting IK and FK calculations...")
    for idx, point in enumerate(grid_points):
        if idx % 100 == 0:
            print(f"Processing point {idx}/{len(grid_points)}...")
        
        # Calculate IK
        ik_solution = inverse_kinematics(panda, point, target_orientation, default_pose)
        ik_solution = ik_solution[:7]  # Exclude the last gripper joint

        # Set robot to the IK solution pose
        for i in range(len(ik_solution)):
            p.resetJointState(panda, i, ik_solution[i])

        # Check for self-collision
        if self_collision(panda):
            continue
        # print(f"No self-collision detected at point {idx}.")

        # Validate with FK
        fk_position, _ = forward_kinematics(panda, ik_solution)
        
        # Check if the FK position is within the error margin of the target position
        if np.linalg.norm(fk_position - point) <= position_error_margin:
            valid_positions.append(point)

    valid_positions = np.array(valid_positions)
    print(f"Valid positions found: {len(valid_positions)}")

    # Plot the valid task space positions
    plot_task_space_positions(valid_positions)

    # Save valid positions to a CSV file
    if input("Save valid positions to CSV? (y/N): ").lower() == 'y':
        directory = 'res/images/'
        filename = 'task_space_samples5.csv'
        np.savetxt(directory + filename, valid_positions, delimiter=',')
        print(f"Valid positions saved to {filename}")

    input("Press Enter to exit...")
    p.disconnect()


# Check individual IK solutions
def create_sphere(position, color, radius=0.05):
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    sphere_id = p.createMultiBody(baseVisualShapeIndex=visual_shape_id, basePosition=position)
    return sphere_id

def main2():    
    # Load valid task space positions
    directory = 'res/images/'
    filename = 'task_space_samples.csv'
    valid_positions = np.loadtxt(directory + filename, delimiter=',')

    # # Plot the valid task space positions
    # plot_task_space_positions(valid_positions)
    # print(valid_positions)
    
    # Take random sample and check IK solution
    random_sample = valid_positions[10]
    print(f"Random sample: {random_sample}")

    # Initialize PyBullet with cleaner settings
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    # Load Panda URDF
    print("Loading Panda URDF...")
    panda_urdf_path = "franka_panda/panda.urdf"
    panda = p.loadURDF(panda_urdf_path, useFixedBase=True)

    # Get the number of joints
    num_joints = p.getNumJoints(panda)
    print(f"Panda has {num_joints} joints.")

    # for joint_index in range(num_joints):
    #     joint_info = p.getJointInfo(panda, joint_index)
    
    #     # joint_info is a tuple containing:
    #     # 0: Joint index
    #     # 1: Joint name
    #     # 2: Joint type (e.g., p.JOINT_PRISMATIC, p.JOINT_REVOLUTE)
    #     # 3: Parent link index
    #     # 4: Child link index
    #     # 5: Joint position (0 if fixed)
    #     # 6: Joint velocity (0 if fixed)
    #     # 7: Joint torque
    #     # 8: Joint lower limit
    #     # 9: Joint upper limit
    #     # 10: Joint max torque
    #     # 11: Joint max velocity
    #     # 12: Joint damping
    #     # 13: Joint friction
        
    #     print(f"Joint {joint_index}:")
    #     print(f"  Name: {joint_info[1].decode('utf-8')}")
    #     print(f"  Type: {joint_info[2]}")
    #     print(f"  Lower Limit: {joint_info[8]}")
    #     print(f"  Upper Limit: {joint_info[9]}")
    #     # print(f"  Max Torque: {joint_info[10]}")
    #     # print(f"  Max Velocity: {joint_info[11]}")
    #     # print(f"  Damping: {joint_info[12]}")
    #     # print(f"  Friction: {joint_info[13]}")
    #     print()

    # # Get link information for all links
    # for link_index in range(num_joints):
    #     # Get link info
    #     link_info = p.getLinkInfo(panda, link_index)
        
    #     # link_info is a tuple containing:
    #     # 0: Link index
    #     # 1: Link name
    #     # 2: Link type (e.g., p.LINK_FIXED, p.LINK_PRISMATIC, etc.)
    #     # 3: Parent joint index
    #     # 4: Joint type
    #     # 5: Joint position (0 if fixed)
    #     # 6: Joint velocity (0 if fixed)
    #     # 7: Joint torque
    #     # 8: Joint lower limit
    #     # 9: Joint upper limit
    #     # 10: Joint max torque
    #     # 11: Joint max velocity
    #     # 12: Joint damping
    #     # 13: Joint friction
        
    #     print(f"Link {link_index}:")
    #     print(f"  Name: {link_info[1].decode('utf-8')}")
    #     print(f"  Type: {link_info[2]}")
    #     print(f"  Parent Joint Index: {link_info[3]}")
    #     print(f"  Lower Limit: {link_info[8]}")
    #     print(f"  Upper Limit: {link_info[9]}")
    #     print(f"  Max Torque: {link_info[10]}")
    #     print(f"  Max Velocity: {link_info[11]}")
    #     print(f"  Damping: {link_info[12]}")
    #     print(f"  Friction: {link_info[13]}")
    #     print()

    # Define the default pose and orientation
    default_pose = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
    target_orientation = [1, 0, 0, 0]  # End effector pointing downwards

    # Calculate IK
    ik_solution = inverse_kinematics(panda, random_sample, target_orientation, default_pose)
    print(f"IK solution: {ik_solution}")

    # Set robot to the IK solution pose
    for i in range(7):
        p.resetJointState(panda, i, ik_solution[i])

    # Validate with FK
    fk_position, fk_orientation = forward_kinematics(panda, ik_solution[:7])
    print(f"FK position: {fk_position}")
    print(f"FK orientation: {fk_orientation}")

    # Place spheres for visualization
    create_sphere(fk_position, color=[1, 0, 0, 1])  # Red sphere at FK position
    create_sphere(random_sample, color=[0, 1, 0, 1])  # Green sphere at target position

    # Error calculation
    position_error = np.linalg.norm(fk_position - random_sample)
    orientation_error = target_orientation - fk_orientation
    print(f"Position error: {position_error}")
    print(f"Orientation error: {orientation_error}")

    # Keep the simulation running to visualize
    p.setRealTimeSimulation(1)
    input("Press Enter to exit...")
    p.disconnect()


# Plot projections of samples on XY, XZ, and YZ planes
def main3():
    # Load valid task space positions
    directory = 'res/images/'
    filename = 'task_space_samples4.csv'
    valid_positions = np.loadtxt(directory + filename, delimiter=',')
    
    # Plot the XY plane
    plt.figure()
    plt.scatter(valid_positions[:, 0], valid_positions[:, 1], c='b', marker='o', s=10)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.gca().set_aspect('equal', adjustable='box')
    # plt.title('XY Plane Projection')
    plt.show()

    # Plot the XZ plane
    plt.figure()
    plt.scatter(valid_positions[:, 0], valid_positions[:, 2], c='b', marker='o', s=10)
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.gca().set_aspect('equal', adjustable='box')
    # plt.title('XZ Plane Projection')
    plt.show()

    # Plot the YZ plane
    plt.figure()
    plt.scatter(valid_positions[:, 1], valid_positions[:, 2], c='b', marker='o', s=10)
    plt.xlabel('Y')
    plt.ylabel('Z')
    plt.gca().set_aspect('equal', adjustable='box')
    # plt.title('YZ Plane Projection')
    plt.show()


# Connecting samples to form roadmap
def modified_homogeneous_transform(a, d, alpha, theta):
    """ Compute the modified Denavit-Hartenberg transformation matrix """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -sa*d],
        [st*sa, ct*sa, ca, ca*d],
        [0, 0, 0, 1]
    ])

def positions_from_fk(base_position, base_orientation, q, dh_params):
    """ Extract the positions of the DH frames using forward kinematics"""
    #  Initialize the positions list
    positions = []

    # Initialize the transformation matrix with base position and orientation
    base_rot = R.from_quat(base_orientation).as_matrix()
    T_base = np.eye(4)
    T_base[:3, :3] = base_rot
    T_base[:3, 3] = base_position

    # Initialize the transformation matrix
    T = T_base

    #  Compute positions of frames using forward kinematics
    for i, params in enumerate(dh_params):
        if i == 7:
            T_i = modified_homogeneous_transform(params["a"], params["d"], params["alpha"], params["theta"])
        elif i < 7:
            T_i = modified_homogeneous_transform(params["a"], params["d"], params["alpha"], q[i])
        T = np.matmul(T, T_i)
        positions.append(T[:3, 3])
    return positions

def task_space_pose_distance(base_position, base_orientation, q1, q2, dh_params): 
    pos1 = positions_from_fk(base_position, base_orientation, q1, dh_params)
    pos2 = positions_from_fk(base_position, base_orientation, q2, dh_params)
    distance = 0
    for i in range(len(pos1)):
        distance += np.linalg.norm(pos1[i] - pos2[i])
    return distance

def connect_knn(panda, base_position, base_orientation, dh_params, samples, k=6):
    roadmap = {}
    count = 0
    for p in samples: 
        q = np.round(inverse_kinematics(panda, p, [1, 0, 0, 0], [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]), 5)
        
        for i in range(7):
            p.resetJointState(panda, i, q[i])

        count += 1
        if count % 100 == 0:
            print(f"Processing IK point {count}/{len(samples)}...")
        
        roadmap[tuple(q)] = []

    count = 0
    for q1 in roadmap.keys():         
        # Initialize a stack to keep track of nearest neighbors
        nn = []

        for i in range(7):
            p.resetJointState(panda, i, q1[i])

        count += 1
        if count % 100 == 0:
            print(f"Processing NN point {count}/{len(roadmap.keys())}...")

        for q2 in roadmap.keys():
            if np.array_equal(q1, q2):
                continue
                        
            for i in range(7):
                p.resetJointState(panda, i, q2[i])

            # Calculate the task space pose distance between q1 and q2
            d = task_space_pose_distance(base_position, base_orientation, q1, q2, dh_params)
            
            # Maintain a stack of k-nearest neighbors
            if len(nn) < k:
                nn.append((d, tuple(q2)))
            else:
                # Replace the farthest neighbor if current one is closer
                max_d = max(nn, key=lambda x: x[0])[0]
                if d < max_d:
                    nn.remove(max(nn, key=lambda x: x[0]))
                    nn.append((d, tuple(q2)))

        # Add the nearest neighbors to the roadmap
        for _, neighbor in nn:
            if neighbor not in roadmap[tuple(q1)]:
                roadmap[tuple(q1)].append(neighbor)
            if tuple(q1) not in roadmap[neighbor]:
                roadmap[neighbor].append(tuple(q1))

    return roadmap

def plot_roadmap_projections(roadmap): 
    # DH parameters for the Panda robot
    dh_params = [
            {"a": 0, "d": 0.333, "alpha": 0, "theta": 0},
            {"a": 0, "d": 0, "alpha": -np.pi/2, "theta": 0},
            {"a": 0, "d": 0.316, "alpha": np.pi/2, "theta": 0},
            {"a": 0.0825, "d": 0, "alpha": np.pi/2, "theta": 0},
            {"a": -0.0825, "d": 0.384, "alpha": -np.pi/2, "theta": 0},
            {"a": 0, "d": 0, "alpha": np.pi/2, "theta": 0},
            {"a": 0.088, "d": 0, "alpha": np.pi/2, "theta": 0}
            # {"a": 0, "d": 0.107, "alpha": 0, "theta": 0}
        ] 
    
    
    # Plot the XY plane
    plt.figure()
    for node, neighbors in roadmap.items():
        pos1 = positions_from_fk([0, 0, 0], [0, 0, 0, 1], node, dh_params)[-1]
        for neighbor in neighbors:
            pos2 = positions_from_fk([0, 0, 0], [0, 0, 0, 1], neighbor, dh_params)[-1]
            plt.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], 'b-', lw=0.5)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.gca().set_aspect('equal', adjustable='box')
    # plt.title('XY Plane Projection')
    plt.show()

    # Plot the XZ plane
    plt.figure()
    for node, neighbors in roadmap.items():
        pos1 = positions_from_fk([0, 0, 0], [0, 0, 0, 1], node, dh_params)[-1]
        for neighbor in neighbors:
            pos2 = positions_from_fk([0, 0, 0], [0, 0, 0, 1], neighbor, dh_params)[-1]
            plt.plot([pos1[0], pos2[0]], [pos1[2], pos2[2]], 'b-', lw=0.5)
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.gca().set_aspect('equal', adjustable='box')
    # plt.title('XZ Plane Projection')
    plt.show()

    # Plot the YZ plane
    plt.figure()
    for node, neighbors in roadmap.items():
        pos1 = positions_from_fk([0, 0, 0], [0, 0, 0, 1], node, dh_params)[-1]
        for neighbor in neighbors:
            pos2 = positions_from_fk([0, 0, 0], [0, 0, 0, 1], neighbor, dh_params)[-1]
            plt.plot([pos1[1], pos2[1]], [pos1[2], pos2[2]], 'b-', lw=0.5)
    plt.xlabel('Y')
    plt.ylabel('Z')
    plt.gca().set_aspect('equal', adjustable='box')
    # plt.title('YZ Plane Projection')
    plt.show()

def save_roadmap(roadmap, filename):
    # Save roadmap to a CSV file
    if input("Save roadmap to CSV? (y/N): ").lower() == 'y':
        directory = 'res/images/'
        with open(directory + filename, 'w') as f:
            for node, neighbors in roadmap.items():
                s = f"("
                for value in node: 
                    s += f"{value},"
                s = s[:-1] # remove last comma
                s += f"):" # to signify the start of neighbors
                for neighbor in neighbors:
                    s += f"("
                    for value in neighbor: 
                        s += f"{value},"
                    s = s[:-1]
                    s += f"),"
                s = s[:-1] # remove last comma
                s += f";" # to signify the end of neighbors
                f.write(s) 
                f.write("\n")
        print(f"Roadmap saved to {directory + filename}")

def main4():
    # Load valid task space positions
    directory = 'res/images/'
    filename = 'task_space_samples4.csv'
    samples = np.loadtxt(directory + filename, delimiter=',')

    # DH parameters for the Panda robot
    dh_params = [
            {"a": 0, "d": 0.333, "alpha": 0, "theta": 0},
            {"a": 0, "d": 0, "alpha": -np.pi/2, "theta": 0},
            {"a": 0, "d": 0.316, "alpha": np.pi/2, "theta": 0},
            {"a": 0.0825, "d": 0, "alpha": np.pi/2, "theta": 0},
            {"a": -0.0825, "d": 0.384, "alpha": -np.pi/2, "theta": 0},
            {"a": 0, "d": 0, "alpha": np.pi/2, "theta": 0},
            {"a": 0.088, "d": 0, "alpha": np.pi/2, "theta": 0},
            {"a": 0, "d": 0.107, "alpha": 0, "theta": 0}
        ]
    
    # Initialize PyBullet with cleaner settings
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    # Load Panda URDF
    print("Loading Panda URDF...")
    panda_urdf_path = "franka_panda/panda.urdf"
    panda = p.loadURDF(panda_urdf_path, useFixedBase=True)
    base_position = [0, 0, 0]
    base_orientation = [0, 0, 0, 1]

    # Define the default pose and orientation
    default_pose = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
    target_orientation = [1, 0, 0, 0]  # End effector pointing downwards

    # Connect the samples to form a roadmap
    # roadmap = connect_knn(panda, [0, 0, 0], [0, 0, 0, 1], dh_params, valid_positions, k=6)
    roadmap = {}
    k=6
    count = 0
    for pos in samples: 
        q = np.round(inverse_kinematics(panda, pos, target_orientation, default_pose), 5)
        q = tuple(q[:-2]) # remove redundant last two fixed joints and cast to tuple 

        for i in range(len(q)):
            p.resetJointState(panda, i, q[i])

        count += 1
        if count % 100 == 0:
            print(f"Processing IK point {count}/{len(samples)}...")
        
        roadmap[tuple(q)] = []

    count = 0
    for q1 in roadmap.keys():    
        # print('q1: ', q1)
        # input()     
        # Initialize a stack to keep track of nearest neighbors
        nn = []

        for i in range(len(q1)):
            p.resetJointState(panda, i, q1[i])

        count += 1
        if count % 100 == 0:
            print(f"Processing NN point {count}/{len(roadmap.keys())}...")

        for q2 in roadmap.keys():
            # print('q2: ', q2)
            # input()

            if np.array_equal(q1, q2):
                # print(f'equal: {q1} and {q2}')
                continue
                        
            for i in range(len(q2)):
                p.resetJointState(panda, i, q2[i])

            # Calculate the task space pose distance between q1 and q2
            d = task_space_pose_distance(base_position, base_orientation, q1, q2, dh_params)
            # print(f'd: {d}')
            # input()

            # Maintain a stack of k-nearest neighbors
            if len(nn) < k:
                nn.append((d, tuple(q2)))
                # print('nn: ', nn)
                # input()
            else:
                # Replace the farthest neighbor if current one is closer
                max_d = max(nn, key=lambda x: x[0])[0]
                if d < max_d:
                    nn.remove(max(nn, key=lambda x: x[0]))
                    nn.append((d, tuple(q2)))
                    # print('nn: ', nn)
                    # input()

        # print('nn: ', nn)
        # input()

        # Add the nearest neighbors to the roadmap
        for tup in nn:
            # print('tup: ', tup)
            # input()
            d, neighbor = tup
            # print('d: ', d)
            # print('neighbor: ', neighbor)
            # input()
            if neighbor not in roadmap[q1]:
                # print('roadmap[q1]: ', roadmap[q1])
                # input()
                roadmap[tuple(q1)].append(neighbor)
                # print('roadmap[q1]: ', roadmap[q1])
                # input()
            if q1 not in roadmap[neighbor]:
                # print('roadmap[neighbor]: ', roadmap[neighbor])
                # input()
                roadmap[neighbor].append(tuple(q1))
                # print('roadmap[neighbor]: ', roadmap[neighbor])
                # input()

    print(roadmap)
    input()

    # Plot the roadmap projections
    plot_roadmap_projections(roadmap)

    # Save roadmap to a CSV file
    save_roadmap(roadmap, 'task_space_roadmap4.csv')

# Postprocessing
# Define joint limits based on the provided image
joint_limits = {
    'q_max': [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159],
    'q_min': [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
}

invalid_joint_angle_count = 0
invalid_node_set = set()

def recast_joint_within_limits(joint_values):
    global invalid_joint_angle_count
    global invalid_node_set
    """Adjust joint values to be within the specified joint limits, or count them as invalid if not possible."""
    adjusted_values = []
    invalid_found = False
    
    for i, q in enumerate(joint_values):
        original_q = q
        while q > joint_limits['q_max'][i]:
            q -= 2 * np.pi
        while q < joint_limits['q_min'][i]:
            q += 2 * np.pi
        
        # Check if value is still outside limits after adjustments
        if q > joint_limits['q_max'][i] or q < joint_limits['q_min'][i]:
            invalid_joint_angle_count += 1
            invalid_found = True
            print(f"Unvalid joint angle: {original_q} (out of bounds)")
        
        adjusted_values.append(q)
    
    if invalid_found:
        invalid_node_set.add(tuple(adjusted_values))
    
    return tuple(adjusted_values)

def load_roadmap(filename):
    roadmap = {}
    directory = 'res/images/'
    
    # Improved regex to match scientific notation, including negative exponents
    float_pattern = r'-?\d+(?:\.\d+)?(?:[eE]-?\d+)?'

    with open(directory + filename, 'r') as f:
        for line in f:
            line = line.strip()
            
            # Split the line into the node and its neighbors
            node_part, neighbors_part = line.split(":")
            
            # Convert the node part into a tuple of floats and recast within joint limits
            node = tuple(map(float, re.findall(float_pattern, node_part)))
            
            # Split the neighbors part, convert each neighbor into a tuple of floats, and recast
            neighbors = [
                tuple(map(float, re.findall(float_pattern, neighbor)))
                for neighbor in neighbors_part.split('),(')
            ]
            
            # Update the roadmap dictionary
            roadmap[node] = neighbors

    return roadmap

def load_and_recast_roadmap(filename):
    roadmap = {}
    directory = 'res/images/'
    
    # Improved regex to match scientific notation, including negative exponents
    float_pattern = r'-?\d+(?:\.\d+)?(?:[eE]-?\d+)?'

    with open(directory + filename, 'r') as f:
        for line in f:
            line = line.strip()
            
            # Split the line into the node and its neighbors
            node_part, neighbors_part = line.split(":")
            
            # Convert the node part into a tuple of floats and recast within joint limits
            node = tuple(map(float, re.findall(float_pattern, node_part)))
            node = recast_joint_within_limits(node)
            
            # Split the neighbors part, convert each neighbor into a tuple of floats, and recast
            neighbors = [
                recast_joint_within_limits(tuple(map(float, re.findall(float_pattern, neighbor))))
                for neighbor in neighbors_part.split('),(')
            ]
            
            # Update the roadmap dictionary
            roadmap[node] = neighbors

    return roadmap

def main5():
    global invalid_joint_angle_count
    global invalid_node_set
    
    # Load and recast the roadmap
    roadmap = load_and_recast_roadmap('task_space_roadmap2.csv')
    # print("Original Roadmap:", roadmap)
    
    invalid_node_count = len(invalid_node_set)
    print('invlaid node set: ', invalid_node_set)

    print(f"Total invalid joint angles encountered: {invalid_joint_angle_count}")
    print(f"Total invalid nodes encountered: {invalid_node_count}")
    
    # Delete invalid nodes as keys
    for invalid_node in invalid_node_set:
        if invalid_node in roadmap.keys():
            del roadmap[invalid_node]
    
    # Delete invalid nodes as neighbors
    for node, neighbors in roadmap.items():
        roadmap[node] = [neighbor for neighbor in neighbors if neighbor not in invalid_node_set]
    
    # print("Roadmap after deleting invalid nodes and neighbors:", roadmap)
    print(f"Remaining nodes: {len(roadmap.keys())}")

    plot_roadmap_projections(roadmap)

    # Optionally save the adjusted roadmap
    if input("Save adjusted roadmap to CSV? (y/N): ").lower() == 'y':
        save_roadmap(roadmap, 'task_space_roadmap2_adjusted_pruned.csv')

def main6(): 
    # Initialize PyBullet with cleaner settings
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    # Load Panda URDF
    print("Loading Panda URDF...")
    panda_urdf_path = "franka_panda/panda.urdf"
    panda = p.loadURDF(panda_urdf_path, useFixedBase=True)
                                           
    roadmap = load_roadmap('task_space_roadmap2_adjusted.csv')
    
    for node in roadmap.keys(): 
        for i in range(len(node)):
            p.resetJointState(panda, i, node[i])
        p.stepSimulation()
        time.sleep(0.1)

    input()
    p.disconnect()

if __name__ == '__main__':
    # main1()
    # main2()
    # main3()
    # main4()
    main5()
    # main6()