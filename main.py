import os
import subprocess
import pandas
import seaborn as sns
import matplotlib.pyplot as plt
import imageio 
import sys

def execute_file(module_path):
    """
    Execute the selected Python file as a module using `python -m`.
    """
    try:
        # Use the current Python interpreter
        python_executable = sys.executable
        print(f"Using Python interpreter: {python_executable}")
        result = subprocess.run([python_executable, "-m", module_path], check=True)
        print(f"Execution completed with return code: {result.returncode}")
    except subprocess.CalledProcessError as e:
        print(f"Error occurred: {e}")
        print(f"Return code: {e.returncode}")
        print(f"Output: {e.output}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def main():
    print("\n1 - experiments")
    print("2 - simulations")
    print("3 - tests")

    choice = int(input("\nEnter the number of the directory from which you would like to execute a file: "))
    
    if choice == 1:
        directory = "experiments.pybullet"
        print("\n1 - exp_scalability.py")
        print("2 - plot_scalability2.py")
        print("3 - tab_scalability.py")

        file_choice = int(input("\nEnter the number of the file you would like to execute: "))
        if file_choice == 1:
            selected_file = "exp_scalability"
        elif file_choice == 2:
            selected_file = "plot_scalability2"
        elif file_choice == 3:
            selected_file = "tab_scalability"
        else:
            raise ValueError("Invalid file choice. Exiting.")

    elif choice == 2:
        directory = "sims"  # Adjust to your actual path
        print("\n1 - cbs_prm.py")
        print("2 - closest_ik_solution.py")
        print("3 - joint_control.py")
        print("4 - pick_and_place.py")
        print("5 - prioritized_prm.py")
        print("6 - trajectory_execution.py")

        file_choice = int(input("\nEnter the number of the file you would like to execute: "))
        if file_choice == 1:
            selected_file = "cbs_prm"
        elif file_choice == 2:
            selected_file = "closest_ik_solution"
        elif file_choice == 3:
            selected_file = "joint_control"
        elif file_choice == 4:
            selected_file = "pick_and_place"
        elif file_choice == 5:
            selected_file = "prioritized_prm"
        elif file_choice == 6:
            selected_file = "trajectory_execution"
        else:
            raise ValueError("Invalid file choice. Exiting.")
    
    elif choice == 3:
        directory = "tests"  # Adjust to your actual path
        print("\n1 - models")
        print("2 - planners")
        print("3 - robots")
        sub_dir = int(input("\nEnter the number of the subdirectory from which you would like to execute a file: "))
        if sub_dir == 1: 
            sub_directory = "models"
            print("\n1 - test_franka_panda.py")
            print("2 - test_kuka_iiwa.py")
            print("3 - test_obstacle.py")
            print("4 - test_turtlebot.py")
            file_choice = int(input("\nEnter the number of the file you would like to execute: "))
            if file_choice == 1:
                selected_file = "test_franka_panda"
            elif file_choice == 2:
                selected_file = "test_kuka_iiwa"
            elif file_choice == 3:
                selected_file = "test_obstacle"
            elif file_choice == 4:
                selected_file = "test_turtlebot"
            else:
                raise ValueError("Invalid file choice. Exiting.")
        elif sub_dir == 2:
            sub_directory = "planners.prm.pybullet"
            print("\n1 - test_cbsprm.py")
            print("2 - test_prioritized_prm.py")
            print("3 - test_degree_prm.py")
            print("4 - test_distance_prm.py")
            file_choice = int(input("\nEnter the number of the file you would like to execute: "))
            if file_choice == 1:
                selected_file = "test_cbsprm"
            elif file_choice == 2:
                selected_file = "test_prioritized_prm"
            elif file_choice == 3:
                selected_file = "test_degree_prm"
            elif file_choice == 4:
                selected_file = "test_distance_prm"
            else:
                raise ValueError("Invalid file choice. Exiting.")
        elif sub_dir == 3:
            sub_directory = "robots"
            print("\n1 - test_panda.py")
            print("2 - test_planar_arm.py")
            print("3 - test_robot.py")
            file_choice = int(input("\nEnter the number of the file you would like to execute: "))
            if file_choice == 1:
                selected_file = "test_panda"
            elif file_choice == 2:
                selected_file = "test_planar_arm"
            elif file_choice == 3:
                selected_file = "test_robot"
            else:
                raise ValueError("Invalid file choice. Exiting.")
        else:
            raise ValueError("Invalid subdirectory choice. Exiting.")  
    else:
        raise ValueError("Invalid directory choice. Exiting.")

    # Construct the module path and execute the file
    module_path = f"{directory}.{sub_directory}.{selected_file}" if choice == 3 else f"{directory}.{selected_file}"
    print(f"\nExecuting {module_path}...\n")
    execute_file(module_path)

if __name__ == "__main__":
    main()


