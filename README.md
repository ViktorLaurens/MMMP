# MMMP (Multi-Manipulator Motion Planning)

<!--  -->
## Introduction
This repository is part of a final dissertation project for a Master of Science in Electromechanical Engineering, specializing in Mechatronics and Robotics. The project centers on advanced motion planning techniques for robotic manipulators, specifically integrating Conflict-Based Search (CBS) with Probabilistic RoadMaps (PRM).

The primary objective of this research is to develop a novel motion planning algorithm that merges the strengths of coupled and decoupled approaches. This hybrid method aims to achieve efficient, collision-free motion planning for multiple manipulators operating in close proximity, particularly in scenarios involving pick-and-place tasks. To facilitate its development, this versatile multi-robot motion planning library has been created.

While the focus is on manipulators performing pick-and-place operations, the framework is designed to be adaptable for various robotic applications and scenarios. Additionally, this project provides a foundation for benchmarking the new hybrid approach against traditional coupled and decoupled methods, offering valuable insights into its performance and advantages.

<!--  -->
## Features
### Hybrid motion planning algorithm: CBS-PRM
The CBS-PRM algorithm combines Conflict-Based Search (CBS) with Probabilistic RoadMaps (PRM) to provide efficient, collision-free pathfinding for multiple manipulators.

<div align="center">
  <img width="60%" src="res/gifs/CBSPRM_20240828_123606.gif" alt="CBS-PRM Algorithm Visualization"/> 
</div>

### Python library of multi-robot motion planners
This collection of planners can be found in the [`planners`](planners) directory. Currently, it includes a range of sampling-based path planners, categorized as follows:

- **Graph-Based Planners:**
  - **Distance-PRM** (coupled)
  - **Degree-PRM** (coupled)
  - **Prioritized-PRM** (decoupled)
  - **CBS-PRM** (hybrid)

- **Tree-Based Planners:**
  - **RRT** (coupled)
  - **Prioritized-RRT** (decoupled)

### Benchmarking and comparison tools
This repository provides a suite of tools for benchmarking and comparing the hybrid motion planning approach (CBS-PRM) against traditional methods. These tools are located in the [`experiments`](experiments) directory. Key features include:
- **Scenario testing:** In the [`exp_scalability`](experiments/pybullet/exp_scalability.py) file, different scenarios are tested in which the scalability performance is observed by tracking different performance metrics.
- **Performance Metrics:** Performance metrics provide insights into the efficiency and performance of different path planning algorithms. These metrics include:
  - Execution time
  - Memory usage
  - Path lengths
  - Success rates
- **Visualization Tools:** Several files are available for plotting and visualizing the performance metrics of different algorithms. These tools help compare and illustrate benchmark results, enhancing the understanding of comparative performance.


### Trajectory generation
The path planner computes a collision-free path in joint space consisting of the sequence of linear edges connecting waypoints. To ensure the path is collision-free, the planner assumes the robots move at a constant speed of equal magnitude along their paths. However, these zig-zag motions are impractical for real-world execution. 

Therefore, a trajectory is generated in a next phase that respects the kinematic and dynamic constraints of the robots while closely following the original zig-zag paths. The computed trajectory uses Linear Segments with Parabolic Blends (LSPBs) to smoothly transition between waypoints, ensuring that the motion remains collision-free.

<div align="center">
  <img width = "60%" src="res/images/PathBeforeTrajectory.png"/> 
  <img width = "60%" src="res/images/PathAndTrajectory.png"/> 
</div>

### Easy Adaptation, Extension, and Integration
The modular structure of this project, along with the abstraction of planners, robots, environments, and tasks, offers the following benefits:
  - **Adaptability:** Easily adapt the programs to utilize different robots for various tasks.
  - **Extendability:** Easily extend the application of robots to operate in diverse environments.
  - **Integrability:** Easily integrate additional path planners.


<!--  -->
## Table of Contents
- [Installation](#installation)
- [Project Structure](#project-structure-overview)
- [Usage](#usage--examples)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)
- [Contact Information](#contact-information)
- [Acknowledgements](#acknowledgements)

<!--  -->
## Installation
1. **Navigate to the desired directory on the local device.**

2. **Clone the repository**:
    ```bash
    git clone https://github.com/ViktorLaurens/MMMP.git
    cd MMMP
    ```

3. **Create and activate a virtual environment (optional)**:
    ```bash
    python -m venv venv
    source venv/bin/activate
    ```

4. **Install dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

<!--  -->
## Project Structure Overview
    MMMP/
    ├── docs/                   # Research paper for this project.
    ├── experiments/            # Experiments to demonstrate the project's capabilities.
    ├── models/                 # URDF files for robot classes. 
    ├── planners/               # Path and motion planning algorithms. 
    ├── res/                    # Resources such as data files, GIFs, images, plots, and videos.
    ├── robots/                 # Robot Python classes. 
    ├── sims/                   # Simulation scripts. 
    ├── tests/                  # Test scripts. 
    ├── utils/                  # Tools and utilities. 
    ├── .gitignore
    ├── LICENSE
    ├── README.md
    ├── main.py
    └── requirements.txt

### docs/ 
Contains the research paper explaining the rationale behind the creation of this multi-robot motion planning library and the underlying algorithms and methodologies involved. See [Documentation](#documentation). 

### experiments/
Contains files for benchmarking and comparing the hybrid motion planning approach (CBS-PRM) against traditional methods. See [Features: Benchmarking and comparison tools](#benchmarking-and-comparison-tools). 

### models/
The [`models`](models) directory contains URDF (Unified Robot Description Format) files that are essential for various aspects of robot simulation and interaction in a Pybullet environment. These files are used for:
- **Visualization:** The URDF files provide detailed descriptions of robot and object models, which are used to render accurate visual representations in the simulation. This helps in visualizing the robots and objects in a realistic manner.
- **Collision Checking:** The URDF files define the geometries and collision properties of robots and objects, enabling the simulation to perform collision detection and ensure that robots navigate through the environment without intersecting with obstacles.
- **Physics Simulation:** The URDF files include information about the physical properties of robots and objects, such as mass, inertia, and joint dynamics. This data is crucial for realistic physics simulations, allowing the simulation to model the behavior and interactions of robots and objects under various forces and constraints.

### planners/
The [`planners`](planners) directory contains implementations of various path planning algorithms used to determine collision-free paths for multiple robots. In this context, collision-free means free from collisions with other robots, static obstacles in the workspace and of course free from collisions between different links of the same robot. See [Features](#features).

### res/
The [`res`](res) directory is dedicated to storing various resources that support the functionality of the project. This includes:
- **Data:** Important files containing data of metrics for comparing the performance of different algorithms gathered from scalability experiments.
- **GIFs:** Visual assets used for illustrative purposes. 
- **Images:** Visual assets used for documentation, visualization, or analysis.
- **Plots:** Visual assets used for visualizing data.
- **Videos:** Media files that provide demonstrations and simulations.

### robots/
The [`robots`](robots) directory contains Python classes for defining and managing various robot models in Pybullet simulations. This includes: 
- **Franka Emika Panda Manipulator:** A detailed class for the Franka Emika Panda robot, which is a versatile robotic manipulator, widely-used for research purposes.
- **Planar Arm:** A class for a simple planar arm with configurable link counts and lengths, designed for basic simulations in 2D.

### sims/
The [`sims`](sims) directory includes scripts for running simulations that showcase the capabilities and functionalities of this repository.

### tests/
The [`tests`](tests) directory features scripts for validating the functionality and performance of various components within the codebase. These tests ensure that different parts of the system work correctly and efficiently, helping to identify and resolve issues during development. They are not meant to be used anymore after development. 

### utils/
The [`utils`](utils) directory houses a range of tools and utilities that support the core functionalities of the project. This includes functions that facilitate various tasks ranging from inverse kinematics calculations and trajectory computation to tools for setting up and managing Pybullet simulations.


<!--  -->
## Usage & examples
1. **Run the main script**:
    ```bash
    python -m main
    ```


<!--  -->
## Documentation
For a comprehensive understanding of the rationale behind the creation of this multi-robot motion planning library and the underlying algorithms and methodologies involved, you can access the research paper from the docs directory and download the paper from there. Use the link below to be redirected to the file and download it.

[`Download the Research Paper (PDF)`](docs/MMMP_research_paper.pdf)


<!--  -->
## Contributing
We welcome contributions to improve this project. Follow these steps to contribute:

1. **Fork the repository**.
   - Go to the repository on GitHub: `https://github.com/ViktorLaurens/MMMP`.
   - Click the "Fork" button to copy the repository to your GitHub account.

2. **In the desired directory, clone your fork in the desired directory**:
    ```bash
    git clone https://github.com/ViktorLaurens/MMMP.git
    cd MMMP
    ```

3. **Create a new topic branch**:
    ```bash
    git checkout -b my-topic-branch
    ```

4. **Make your changes**.
   - Make your changes to the codebase using your preferred editor.
   - Ensure your changes follow the project's coding standards and guidelines.
   - Run tests to make sure your changes don't break existing functionality.

5. **Commit your changes**:
    ```bash
    git add .
    git commit -m "Add detailed description of the feature or fix"
    ```

6. **Push to the branch**:
    ```bash
    git push origin my-topic-branch
    ```

7. **Create a pull request**.
   - Go to your forked repository on GitHub.
   - Click the "Compare & pull request" button.
   - Provide a clear title and description for your pull request, explaining what changes you made and why.
   - Submit the pull request.

8. **Respond to review comments**.
   - The project maintainers will review your pull request.
   - Be prepared to make additional changes based on their feedback.
   - Once all feedback is addressed and your changes are approved, your pull request will be merged.

<!--  -->
## License
This project is licensed under the MIT License.

<!--  -->
## Contact Information
- Author: Viktor Laurens De Groote
- Email: viktor.degroote@gmail.com

<!--  -->
## Acknowledgements
- [OMPL Library](https://ompl.kavrakilab.org/) - For providing an excellent base for sampling-based motion planning algorithms.
- [Robotics Toolbox for Python](https://petercorke.com/toolboxes/robotics-toolbox/) - For various robot models and simulation utilities.
- Special thanks to my promotor, Prof. dr. ir. Bram Vanderborght, and my supervisors, dr. ir. Gaoyuan Liu and Prof. dr. ir. Ilias El Makrini, for their guidance and support throughout the project.

<!-- Badges -->
![GitHub last commit](https://img.shields.io/github/last-commit/ViktorLaurens/MMMP)
![GitHub issues](https://img.shields.io/github/issues/ViktorLaurens/MMMP)
![GitHub license](https://img.shields.io/github/license/ViktorLaurens/MMMP)