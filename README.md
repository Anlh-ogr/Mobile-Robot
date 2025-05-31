# Last Bot Simulation

A Pygame-based simulation of a robot navigating a maze using a Particle Swarm Optimization (PSO) algorithm combined with a neural network to reach a goal. The project demonstrates autonomous robot navigation, sensor-based obstacle avoidance, and path optimization.

## Table of Contents
- [Last Bot Simulation](#last-bot-simulation)
  - [Table of Contents](#table-of-contents)
  - [Project Overview](#project-overview)
  - [Features](#features)
  - [Requirements](#requirements)
  - [Installation](#installation)
  - [Project Structure](#project-structure)
  - [How It Works](#how-it-works)
    - [1. **Environment**](#1-environment)
    - [2. **Robot**](#2-robot)
    - [3. **Neural Network**](#3-neural-network)
    - [4. **PSO (Particle Swarm Optimization)**](#4-pso-particle-swarm-optimization)
    - [5. **Simulation**](#5-simulation)
    - [6. **Replay**](#6-replay)
  - [Running the Simulation](#running-the-simulation)
  - [Output Files](#output-files)
  - [Contributing](#contributing)
  - [License](#license)

## Project Overview
The **Last Bot Simulation** is a Python project that simulates a robot navigating through a maze from a starting point to a goal. The robot uses a neural network to process sensor inputs and control wheel velocities, optimized by a PSO algorithm to find the best path. The simulation visualizes the robot's movement, sensor readings, and trail in real-time using Pygame, and saves the best path for replay.

## Features
- **Robot Navigation**: A differential-drive robot navigates a 2D maze using sensor data.
- **Neural Network**: A feedforward neural network processes sensor inputs (six distance sensors and orientation) to control wheel velocities.
- **PSO Optimization**: Particle Swarm Optimization tunes neural network weights to minimize a fitness function based on crash avoidance, rotation, stuck time, sensor distances, and exploration.
- **Visualization**: Real-time Pygame display of the robot, its sensors, movement trail, and start/end points.
- **Path Replay**: Saves and replays the best path found by PSO.
- **Fitness Metrics**: Tracks crashes, total rotation, time stuck, sensor distances, and explored cells.

## Requirements
- **Python 3.8+**
- **Libraries**:
  - `pygame` (for visualization and simulation)
  - `numpy` (for matrix operations in neural network and PSO)
- **Assets**:
  - Maze image (`m8.png`) in a `map` directory
  - Robot image (`bot_right.png`) in an `access` directory

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/last-bot-simulation.git
   cd last-bot-simulation
   ```

2. **Install Dependencies**:
   ```bash
   pip install pygame numpy
   ```

3. **Prepare Assets**:
   - Place the maze image (`m8.png`) in a `map` subdirectory.
   - Place the robot image (`bot_right.png`) in an `access` subdirectory.
   - Ensure the directory structure matches the [Project Structure](#project-structure) below.

## Project Structure
```
last-bot-simulation/
├── access/
│   └── bot_right.png    # Robot image (21x21 pixels after scaling)
├── map/
│   └── m8.png           # Maze image (555x650 pixels)
├── work.py              # Main simulation script
├── best_path.json       # Output: Best path data (generated)
├── replay_path.json     # Output: Replay path data (generated)
└── README.md            # This file
```

- **`work.py`**: Contains the core simulation logic, including `Environment`, `Robot`, `NeuralNetwork`, `PSO`, and `Simulation` classes.
- **`bot_right.png`**: Image of the robot, scaled to 21x21 pixels.
- **`m8.png`**: Maze image, scaled to 555x650 pixels, where black pixels represent walls.
- **`best_path.json`**: Stores the best path found by PSO.
- **`replay_path.json`**: Stores the path taken during replay.

## How It Works
The simulation consists of several components:

### 1. **Environment**
- Defines the maze (555x650 pixels) with a start point (green rectangle) and goal (red rectangle).
- Draws the robot's trail, sensor lines, and coordinate axes (blue for x-axis, green for y-axis).
- Uses Pygame to render the maze and robot visuals.

### 2. **Robot**
- A differential-drive robot with two wheels, controlled by a neural network.
- Six sensors detect distances to walls at angles (0°, 60°, 120°, 180°, 240°, 300° relative to the robot's orientation).
- Tracks position, orientation, wheel velocities, and fitness metrics (e.g., crashes, stuck time, rotation).
- Moves using forward kinematics based on wheel velocities, with collision detection against maze walls.

### 3. **Neural Network**
- A simple feedforward neural network with:
  - **Input Layer**: 7 inputs (6 normalized sensor readings + normalized orientation).
  - **Hidden Layer**: 10 neurons with tanh activation.
  - **Output Layer**: 2 outputs (left and right wheel velocities).
- Weights are optimized by PSO.

### 4. **PSO (Particle Swarm Optimization)**
- Population size: 100 particles.
- Each particle represents a set of neural network weights (90 parameters: 7x10 input-to-hidden + 10x2 hidden-to-output).
- Fitness function penalizes:
  - Crashes (`alpha = 100`).
  - Total rotation (`beta = 0.1`).
  - Time stuck (`delta = 40` for >2 seconds).
  - Proximity to walls (`sensor_weight = 0.5`).
  - Rewards exploration (`epsilon = 2.0` for unique grid cells visited).
- Updates particle positions and velocities using inertia (`0.9`), cognitive (`0.5`), and social (`0.5`) coefficients.

### 5. **Simulation**
- Runs up to 10,000 iterations or until a robot reaches the goal.
- Each iteration:
  - Resets robots to the start.
  - Runs until all robots crash, reach the goal, or exceed 60 seconds.
  - Updates PSO based on fitness scores.
- Saves the best path to `best_path.json` and replays it using the best neural network weights.

### 6. **Replay**
- Replays the best path using a single robot with the optimized neural network weights.
- Saves the replay path to `replay_path.json`.

## Running the Simulation
1. Ensure the `map` and `access` directories contain `m8.png` and `bot_right.png`, respectively.
2. Run the script:
   ```bash
   python work.py
   ```
3. The simulation will:
   - Display the maze and robots in real-time.
   - Print iteration numbers and best fitness scores to the console.
   - Stop when a robot reaches the goal or after 10,000 iterations.
   - Save the best path to `best_path.json`.
   - Replay the best path and save it to `replay_path.json`.

## Output Files
- **`best_path.json`**: Contains a list of dictionaries with the best robot's state at each time step:
  ```json
  [
      {
          "x_position": float,
          "y_position": float,
          "theta_position": float,
          "left_wheel": float,
          "right_wheel": float,
          "elapsed_time": float
      },
      ...
  ]
  ```
- **`replay_path.json`**: Similar format, containing the replayed path.

## Contributing
Contributions are welcome! To contribute:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/your-feature`).
3. Commit changes (`git commit -m "Add your feature"`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a pull request.

Please include tests and update documentation as needed.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

Built with ❤️ using Python, Pygame, and NumPy.