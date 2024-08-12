# LSPB curves
from matplotlib import pyplot as plt
import numpy as np
import seaborn as sns

def clip_waypoints_to_limits(waypoints, joint_limits):
    """
    Ensure that each waypoint is within the joint limits of the robot.
    """
    q_min = np.array(joint_limits['q_min'])
    q_max = np.array(joint_limits['q_max'])
    
    # Clip the waypoints to the joint limits
    clipped_waypoints = np.clip(waypoints, q_min, q_max)
    
    return clipped_waypoints

def calculate_segment_norms(waypoints):
    """
    Calculate the Euclidean norm of each line segment in joint space.
    """
    norms = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    return norms

def calculate_total_path_norm(segment_norms):
    """
    Calculate the total norm of the path as the sum of segment norms.
    """
    return np.sum(segment_norms)

def calculate_time_allocation(segment_norms, total_norm, t_final):
    """
    Calculate delta_t for each segment based on its norm and the total norm.
    """
    delta_t = (segment_norms / total_norm) * t_final
    return delta_t

def calculate_average_speeds(waypoints, delta_t):
    """
    Calculate the average speed in joint space for each joint along each segment.
    """
    joint_differences = np.diff(waypoints, axis=0)  # Differences between consecutive waypoints
    average_speeds = joint_differences / delta_t[:, np.newaxis]
    return average_speeds

def scale_velocities(average_speeds, vmax, limit_factor=0.9):
    """
    Scale velocities to ensure none exceed vmax, or to optimize speed.
    """
    scaling_factors = vmax / np.abs(average_speeds)
    min_scaling_factor = np.min(scaling_factors)
    if min_scaling_factor < 1:
        scaled_speeds = average_speeds * min_scaling_factor * limit_factor  # 9/10th of the limit
    else:
        scaled_speeds = average_speeds  # Already within limits, no scaling
    
    return scaled_speeds

class LSPB:
    def __init__(self, delta_t, t1, p1, v1, a1, v12, p2, v2, a2):
        # Initialize the parameters
        self.t1 = t1
        self.p1 = p1
        self.v1 = v1
        self.a1 = a1
        self.t1_prime = None
        self.p1_prime = None
        self.v12 = v12
        self.t2_prime = None
        self.p2_prime = None
        self.t2 = None
        self.p2 = p2
        self.v2 = v2
        self.a2 = a2

        # Blend parameters
        self.b1 = None
        self.c1 = None
        self.b2 = None
        self.c2 = None
        
        # Calculate the blend points and times
        self._calculate_lspb(delta_t, self.t1, self.p1, self.v1, self.a1, self.v12, self.p2, self.v2, self.a2)

    def _calculate_lspb(self, delta_t, t1, p1, v1, a1, v12, p2, v2, a2):
        # First parabolic blend
        b1 = v1
        c1 = p1
        
        # Time at which first blend ends and linear segment begins
        t1_prime = t1 if a1==0 else (v12-b1)/a1 + t1
        
        # Position at the end of the first parabolic blend
        p1_prime = a1/2 * (t1_prime - t1)**2 + b1 * (t1_prime - t1) + c1
        
        # Second parabolic blend duration
        delta_t2 = 0 if a2==0 else (v2 - v12) / a2
    
        # Time at which the second blend starts
        t2_prime = t1_prime + delta_t if v12==0 else (p2 - p1_prime - (a2/2)*delta_t2**2 - v12*delta_t2 + v12*t1_prime) / v12
     
        # Position at the end of the linear segment (start of second parabolic blend)
        p2_prime = v12 * (t2_prime - t1_prime) + p1_prime
      
        # Time at which the second blend ends
        t2 = t2_prime + delta_t2

        # Position at the end of the second parabolic blend
        b2 = v12
        c2 = p2_prime
        p2 = a2/2*delta_t2**2 + b2*delta_t2 + c2

        # Update the parameters
        self.t1_prime = t1_prime
        self.p1_prime = p1_prime
        self.t2_prime = t2_prime
        self.p2_prime = p2_prime
        self.t2 = t2
        self.p2 = p2
        self.b1 = b1
        self.c1 = c1
        self.b2 = b2
        self.c2 = c2
        return

    def get_trajectory_parameters(self):
        return {
            't1': self.t1, 'p1': self.p1, 'v1': self.v1, 'a1': self.a1,
            't1_prime': self.t1_prime, 'p1_prime': self.p1_prime, 'v12': self.v12,
            't2_prime': self.t2_prime, 'p2_prime': self.p2_prime,
            't2': self.t2, 'p2': self.p2, 'v2': self.v2, 'a2': self.a2
        }

    def __repr__(self):
        params = self.get_trajectory_parameters()
        return (f"LSPB(t1={params['t1']:.3f}, p1={params['p1']:.3f}, v1={params['v1']:.3f}, a1={params['a1']:.3f}, "
                f"t1_prime={params['t1_prime']:.3f}, p1_prime={params['p1_prime']:.3f}, v12={params['v12']:.3f}, "
                f"t2_prime={params['t2_prime']:.3f}, p2_prime={params['p2_prime']:.3f}, t2={params['t2']:.3f}, "
                f"p2={params['p2']:.3f}, v2={params['v2']:.3f}, a2={params['a2']:.3f})")
    
    # Visualize
    def evaluate_position(self, t):
        # Evaluate position at a given time t
        if self.t1 <= t < self.t1_prime:
            # First parabolic blend
            return self.a1/2 * (t - self.t1)**2 + self.b1 * (t - self.t1) + self.c1
        elif self.t1_prime <= t < self.t2_prime:
            # Linear segment
            return self.v12 * (t - self.t1_prime) + self.p1_prime
        elif self.t2_prime <= t < self.t2:
            # Second parabolic blend
            return self.a2/2 * (t - self.t2_prime)**2 + self.b2 * (t - self.t2_prime) + self.c2
        else:
            return None  # Out of bounds

    def evaluate_velocity(self, t):
        # Evaluate velocity at a given time t
        if t < self.t1_prime:
            # First parabolic blend
            return self.a1 * (t - self.t1) + self.b1
        elif t < self.t2_prime:
            # Linear segment
            return self.v12
        elif t <= self.t2:
            # Second parabolic blend
            return self.a2 * (t - self.t2_prime) + self.b2
        else:
            return None # Out of bounds

    def evaluate_acceleration(self, t):
        # Evaluate acceleration at a given time t
        if t < self.t1_prime:
            # First parabolic blend
            return self.a1
        elif t < self.t2_prime:
            # Linear segment
            return 0
        elif t <= self.t2:
            # Second parabolic blend
            return self.a2
        else:
            return None # Out of bounds

    def plot(self):
        # Time range for plotting
        time_range = np.linspace(self.t1, self.t2, num=500)
        
        # Evaluate the position at each time step
        positions = [self.evaluate_position(t) for t in time_range]

        # Set the Seaborn style
        sns.set(style="whitegrid")

        # Plotting
        plt.figure(figsize=(8, 4))
        
        # Plotting the LSPB trajectory
        plt.plot(time_range, positions, label='LSPB Trajectory', color='black', lw=2)
        
        # Scatter plot for the key points
        plt.scatter([self.t1, self.t1_prime, self.t2_prime, self.t2], 
                    [self.p1, self.p1_prime, self.p2_prime, self.p2], 
                    color='red', edgecolor='black', s=20, zorder=5, label='Key Points')

        # Customizing labels and title
        plt.xlabel('Time [s]', fontsize=14)
        plt.ylabel('Position [units]', fontsize=14)
        # plt.title('LSPB Trajectory', fontsize=16)

        # Enhancing the grid and adding a legend
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend(fontsize=12)
        
        # Tight layout for better fit
        plt.tight_layout()

        # Show plot
        plt.show()


def calculate_lspb_trajectory(waypoints, t_final, effort_factor, joint_limits, velocity_limits, acceleration_limits):
    # Clip the waypoints to ensure they are within the joint limits
    waypoints = clip_waypoints_to_limits(waypoints, joint_limits)

    # Step 1: Calculate norms for each line segment
    segment_norms = calculate_segment_norms(waypoints)

    # Step 2: Calculate the total norm of the path
    total_norm = calculate_total_path_norm(segment_norms)

    # Step 3: Select an initial t_final and calculate delta_t for each segment
    delta_t = calculate_time_allocation(segment_norms, total_norm, t_final)
    
    # Step 4: Calculate average speeds for each segment
    average_velocities = calculate_average_speeds(waypoints, delta_t)

    # Step 5: Scale velocities to meet joint limits
    average_velocities = scale_velocities(average_velocities, velocity_limits, limit_factor=0.9)

    # Create LSPB trajectory for each joint
    num_joints = waypoints.shape[1]
    trajectories = {joint: [] for joint in range(num_joints)} 

    for joint in range(num_joints):
        joint_waypoints = waypoints[:, joint]
        segments = []
        n = len(joint_waypoints)

        # First segment
        t1 = 0
        p1 = joint_waypoints[0]
        p2 = joint_waypoints[1]
        v12 = average_velocities[0][joint]
        v1 = 0
        v2 = (average_velocities[0][joint] + average_velocities[1][joint]) / 2
        a = np.min(acceleration_limits) * effort_factor
        a1 = a if v12 > v1 else (-a if v12 < v1 else 0)
        a2 = -a if v12 > v2 else (a if v12 < v2 else 0)
        first_segment = LSPB(delta_t=delta_t[0], t1=t1, p1=p1, v1=v1, a1=a1, v12=v12, p2=p2, v2=v2, a2=a2)
        segments.append(first_segment)

        # Other segments
        for i in range(1, n - 1):
            p1 = segments[-1].p2
            p2 = joint_waypoints[i + 1]
            v12 = average_velocities[i][joint]
            v1 = (average_velocities[i - 1][joint] + average_velocities[i][joint]) / 2
            v2 = 0 if i == n - 2 else (average_velocities[i][joint] + average_velocities[i + 1][joint]) / 2
            a = np.min(acceleration_limits) * effort_factor
            a1 = a if v12 > v1 else (-a if v12 < v1 else 0)
            a2 = -a if v12 > v2 else (a if v12 < v2 else 0)

            # Calculate t1 for the first segment or use the end time of the previous segment
            t1 = segments[-1].t2

            # Create the LSPB segment
            segment = LSPB(delta_t=delta_t[i], t1=t1, p1=p1, v1=v1, a1=a1, v12=v12, p2=p2, v2=v2, a2=a2)
            segments.append(segment)
        
        trajectories[joint] = segments

    return trajectories

def joint_positions_from_trajectories(trajectories, t):
    joint_positions = []
    for joint in trajectories.keys(): 
        lspb_trajectory = trajectories[joint]
        for lspb in lspb_trajectory:
            if lspb.t1 <= t < lspb.t2:
                joint_positions.append(lspb.evaluate_position(t))
                break
        if t == lspb_trajectory[-1].t2:
            joint_positions.append(lspb_trajectory[-1].p2)
    return np.array(joint_positions)


#  Plot position
def plot_lspb_trajectory(trajectory, joint):
    # Initialize the plot
    sns.set(style="whitegrid")
    plt.figure(figsize=(10, 6))
    
    # Plot each segment of the joint trajectory
    for i, segment in enumerate(trajectory):
        time_range = np.linspace(segment.t1, segment.t2, num=500)
        positions = [segment.evaluate_position(t) for t in time_range]
        plt.plot(time_range, positions, lw=2, label=f'Segment {i+1}')

    # Scatter plot for all key points
    all_times = [seg.t1 for seg in trajectory] + [trajectory[-1].t2]
    all_positions = [seg.p1 for seg in trajectory] + [trajectory[-1].p2]
    plt.scatter(all_times, all_positions, color='red', edgecolor='black', s=20, zorder=5, label='Key Points')

    # Customizing labels and title
    plt.xlabel('Time [s]', fontsize=14)
    plt.ylabel('Position [units]', fontsize=14)
    plt.title(f'Joint {joint + 1} LSPB Trajectory', fontsize=16)

    # Enhancing the grid and adding a legend
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(fontsize=12)
    
    # Tight layout for better fit
    plt.tight_layout()

    # Show plot
    plt.show()


def plot_lspb_trajectory_for_joint(trajectories, joint):
    # Check if the specified joint exists
    if joint not in trajectories:
        raise ValueError(f"Joint {joint} not found in trajectories.")
    trajectory = trajectories[joint]
    plot_lspb_trajectory(trajectory, joint)
    return

# Plot velocity
def plot_velocity_profile(trajectories, joint):
    """
    Plot the velocity profile for a specified joint.
    """
    if joint not in trajectories:
        raise ValueError(f"Joint {joint} not found in trajectories.")

    joint_segments = trajectories[joint]
    
    sns.set(style="whitegrid")
    plt.figure(figsize=(10, 6))
    
    for i, segment in enumerate(joint_segments):
        time_range = np.linspace(segment.t1, segment.t2, num=500)
        velocities = [segment.evaluate_velocity(t) for t in time_range]
        plt.plot(time_range, velocities, lw=2, label=f'Segment {i + 1}')

    all_times = [seg.t1 for seg in joint_segments] + [joint_segments[-1].t2]
    all_velocities = [seg.v1 for seg in joint_segments] + [joint_segments[-1].v2]
    plt.scatter(all_times, all_velocities, color='blue', edgecolor='black', s=20, zorder=5, label='Key Points')

    plt.xlabel('Time [s]', fontsize=14)
    plt.ylabel('Velocity [units/s]', fontsize=14)
    plt.title(f'Joint {joint + 1} Velocity Profile', fontsize=16)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.show()

# Plot acceleration
def plot_acceleration_profile(trajectories, joint):
    """
    Plot the acceleration profile for a specified joint.
    """
    if joint not in trajectories:
        raise ValueError(f"Joint {joint} not found in trajectories.")

    joint_segments = trajectories[joint]
    
    sns.set(style="whitegrid")
    plt.figure(figsize=(10, 6))
    
    for i, segment in enumerate(joint_segments):
        time_range = np.linspace(segment.t1, segment.t2, num=500)
        accelerations = [segment.evaluate_acceleration(t) for t in time_range]
        plt.plot(time_range, accelerations, lw=2, label=f'Segment {i + 1}')

    all_times = [seg.t1 for seg in joint_segments] + [joint_segments[-1].t2]
    all_accelerations = [segment.a1 for segment in joint_segments]
    plt.scatter(all_times[:-1], all_accelerations, color='green', edgecolor='black', s=20, zorder=5, label='Key Points')

    plt.xlabel('Time [s]', fontsize=14)
    plt.ylabel('Acceleration [units/s^2]', fontsize=14)
    plt.title(f'Joint {joint + 1} Acceleration Profile', fontsize=16)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.show()