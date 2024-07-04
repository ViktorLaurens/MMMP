# MMMP (Multi-Manipulator Motion Planning)
This repository is developed as part of a final dissertation to obtain the Master of Science in Electromechanical Engineering with a specialization in Mechatronics and Robotics. The project focuses on sampling-based motion planning of manipulators, particularly combining Conflict-Based Search (CBS) and Probabilistic RoadMaps (PRM).

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)
- [Contact Information](#contact-information)
- [Acknowledgements](#acknowledgements)

## Installation
1. **Clone the repository**:
    ```bash
    git clone https://github.com/yourusername/MMMP.git
    cd MMMP
    ```

2. **Create and activate a virtual environment**:
    ```bash
    python -m venv venv
    # On Windows
    .\venv\Scripts\Activate
    # On macOS/Linux
    source venv/bin/activate
    ```

3. **Install dependencies**:
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

## Features
- Conflict-Based Search (CBS) for multi-manipulator planning.
- Probabilistic RoadMaps (PRM) integration.
- Efficient path planning for multiple robotic arms.

## Configuration
- Explain any configuration options available in the project.
- Example configuration files or settings.

## Project Structure
- `main.py`: Entry point of the application.
- `planner/`: Contains the planning algorithms.
- `utils/`: Utility functions and helpers.
- `tests/`: Unit tests for the project.

## Contributing
1. **Fork the repository**.
2. **Create a new branch**:
    ```bash
    git checkout -b feature-branch
    ```
3. **Commit your changes**:
    ```bash
    git commit -m "Add some feature"
    ```
4. **Push to the branch**:
    ```bash
    git push origin feature-branch
    ```
5. **Create a pull request**.

## License
This project is licensed under the MIT License.

## Contact Information
- Author: Your Name
- Email: your.email@example.com

## Acknowledgements
- Acknowledge any contributors, libraries, or resources that were helpful.