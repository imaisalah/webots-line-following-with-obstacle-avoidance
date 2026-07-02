# Webots Line Following with Obstacle Avoidance

## Overview

This project is an autonomous mobile robot simulation developed in **Webots** using **Python**. The robot follows a predefined line using infrared ground sensors while detecting and avoiding obstacles with proximity sensors. After bypassing an obstacle, it automatically searches for and rejoins the original path.

## Features

- Line following using three ground sensors
- Obstacle detection using front proximity sensors
- Autonomous obstacle avoidance
- Line recovery when the path is lost
- Finite State Machine (FSM) for robot behavior
- Differential drive motor control

## Technologies Used

- Python
- Webots Robot Simulator
- Finite State Machine (FSM)

## Project Structure

```
Robotics Project/
├── controllers/
│   └── line_follower_controller/
│       └── line_follower_controller.py
├── protos/
│   └── TrackOne.proto
└── worlds/
    └── my_robot.wbt
```

## How It Works

The robot operates using a finite state machine with multiple navigation states:

- **Line Following** – Follows the black line using ground sensors.
- **Obstacle Detection** – Detects obstacles using front proximity sensors.
- **Obstacle Avoidance** – Executes a sequence of turns and forward movements to bypass obstacles.
- **Line Recovery** – Searches for and reconnects with the original path if the line is lost.

## How to Run

1. Install Webots.
2. Clone this repository.
3. Open `worlds/my_robot.wbt` in Webots.
4. Run the simulation.

## Future Improvements

- Dynamic path planning
- PID-based line following
- Adaptive obstacle avoidance
- Performance optimization

## Author

**Mai Salah**
