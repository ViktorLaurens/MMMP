# MMMP (Multi-Manipulator Motion Planning)

<!--  -->
## Introduction
This repository is developed as part of a final dissertation to obtain the Master of Science in Electromechanical Engineering with a specialization in Mechatronics and Robotics. The project focuses on sampling-based motion planning of manipulators, particularly combining Conflict-Based Search (CBS) and Probabilistic RoadMaps (PRM).

<!--  -->
## Features

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
    ├── experiments/            # various examples to demonstrate the project's capabilities.
    ├── models/                 # Geometric models for robot classes. 
    ├── planners/               # Path and motion planning algorithms. 
    ├── res/                    # Resources such as data files, images, and videos.
    ├── robots/                 # Robot Python classes. 
    ├── sims/                   # Simulation scripts. 
    ├── tests/                  # Test scripts. 
    ├── utils/                  # Tools and utilities. 
    ├── .gitignore
    ├── LICENSE
    ├── README.md
    ├── main.py
    └── requirements.txt

### experiments/
Contains configuration files for various parts of your project. 

### models/
Contains geometric models for robot classes.

### planners/
Contains path and motion planning algorithms.

### res/
Contains resources such as data files, images, and videos.

### robots/
Contains robot python classes.

### sims/
Contains simulation scripts.

### tests/
Contains test scripts.

### utils/
Contains tools and utilities.


<!--  -->
## Usage & examples
1. **Run the main script**:
    ```bash
    python -m main
    ```


<!--  -->
## Documentation
For a comprehensive understanding of the rationale behind the creation of this multi-robot motion planning library and the underlying algorithms and methodologies involved, you can access the research paper from the [docs directory](docs/). You can also download the paper directly using the link below.

[Download the Research Paper (PDF)](docs/MMMP_research_paper.pdf)


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