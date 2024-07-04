# MMMP (Multi-Manipulator Motion Planning)
This repository is developed as part of a final dissertation to obtain the Master of Science in Electromechanical Engineering with a specialization in Mechatronics and Robotics. The project focuses on sampling-based motion planning of manipulators, particularly combining Conflict-Based Search (CBS) and Probabilistic RoadMaps (PRM).

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
<!-- - [Features](#features) -->
<!-- - [Configuration](#configuration) -->
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)
- [Contact Information](#contact-information)
<!-- - [Acknowledgements](#acknowledgements) -->

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

## Usage
1. **Run the main script**:
    ```bash
    python main.py
    ```

2. **Examples**:
    - Describe any example usage or command-line arguments.

<!-- ## Features
- Conflict-Based Search (CBS) for multi-manipulator planning.
- Probabilistic RoadMaps (PRM) integration.
- Efficient path planning for multiple robotic arms. -->

<!-- ## Configuration
- Explain any configuration options available in the project.
- Example configuration files or settings. -->

## Project Structure
    MMMP/
    ├── config/                 # Configuration files
    ├── docs/                   # Documentation
    ├── examples/               # Example scripts
    ├── models/                 # Geometric models for robot classes
    ├── planners/               # Path and motion planning algorithms
    ├── res/                    # Resources (data, images, videos, ...)
    ├── robots/                 # Robot classes
    ├── sims/                   # Simulation scripts
    ├── tests/                  # Test scripts
    ├── utils/                  # Tools and utilities
    ├── .gitignore
    ├── LICENSE
    ├── README.md
    └── requirements.txt



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

## License
This project is licensed under the MIT License.

## Contact Information
- Author: Viktor Laurens De Groote
- Email: viktor.degroote@gmail.com

<!-- ## Acknowledgements
- Acknowledge any contributors, libraries, or resources that were helpful. -->