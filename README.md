# Maze Solver Robot with SLAM

This project features a two-wheeled self-balancing robot capable of autonomously solving mazes using a Simultaneous Localization and Mapping (SLAM) algorithm. The robot is equipped with a TeraRanger Multiflex ToF sensor array for environmental mapping and an MPU-6050 gyroscope for odometry.

<img src="https://github.com/user-attachments/assets/679be388-7260-4449-b8b9-67d971989837" alt="RobotFinalAssembly1" height="500">

## Features

- **Real-time SLAM**: Integrates robot odometry (gyro, distance) with ToF sensor readings to build and update an environmental map on the fly.
- **Live Data Visualization**: A custom PyQtGraph application provides a rich, interactive 2D plot displaying:
  - The robot's current position and orientation.
  - The robot's historical path.
  - Mapped environmental points detected by the ToF sensors.
- **Wi-Fi Communication**: Establishes a robust Wi-Fi connection for seamless data transmission between the robot and a host PC.
- **Keyboard Control**: Intuitive keyboard commands allow for manual control of the robot's movement (forward, backward, turn left, turn right, stop).
- **Autonomous Maze Solving**: The robot can autonomously navigate and solve mazes using the generated map.

## Demo

https://github.com/user-attachments/assets/0ca18e00-be7d-4444-9bbe-24fa2c01fbb0

## Project Structure

```
.
├── communication_hub_code/  # Arduino code for the communication hub
├── mobile_base_code/        # PlatformIO project for the self-balancing robot
├── tests/                   # Test scripts and code
├── videos/                  # Demo videos
├── .gitignore
├── LICENSE
├── logger_config.py         # Logging configuration
├── main.py                  # Main application entry point
├── mazeSolver.py            # Maze solving logic
├── README.md                # This file
├── requirements.txt         # Python dependencies
└── robotInterface.py        # Interface for robot communication and control
```

## Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/haris-mujeeb/Maze_Solver.git
    cd Maze_Solver
    ```

2.  **Install Python dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

3.  **Robot Setup:**
    -   The `mobile_base_code` directory contains the PlatformIO project for the robot's firmware. You will need to have [PlatformIO](https://platformio.org/) installed to compile and upload the code to the robot.
    -   The `communication_hub_code` directory contains the Arduino code for the communication hub.
    -   Ensure your robot's firmware is configured to:
        -   Send sensor data (ToF, gyro, odometry) over Wi-Fi (TCP/IP) to the `RECEIVE_HOST` and `RECEIVE_PORT` specified in `main.py`.
        -   Listen for movement commands on `SEND_HOST` and `SEND_PORT` specified in `main.py`.

## Usage

1.  **Run the main application:**
    ```bash
    python main.py
    ```

2.  **Control the robot:**
    -   The PyQtGraph application will launch, displaying the robot's navigation map.
    -   Use the following keyboard commands to control the robot:
        -   **W**: Move forward
        -   **S**: Move backward
        -   **A**: Turn left
        -   **D**: Turn right
        -   **R**: Stop
        -   **G**: Go to a manually entered target
    -   You can also left-click on the plot to set a target position for the robot.

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue if you have any suggestions or find any bugs.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.