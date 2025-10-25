# Autonomous Maze Solving Robot with SLAM 🤖

This project features a two-wheeled, self-balancing robot capable of autonomous navigation and maze solving. It uses a host-PC-based **Simultaneous Localization and Mapping (SLAM)** algorithm to build a 2D map of its environment in real-time.

[cite\_start]The robot's hardware is a custom-designed, differential-drive platform based on an **Arduino Nano** for real-time balancing control and an **ESP32-S3** as a central communication hub[cite: 21, 23]. [cite\_start]It uses a **TeraRanger Multiflex** Time-of-Flight (ToF) sensor array for 2D environmental mapping[cite: 19].

\<img src="[https://github.com/user-attachments/assets/679be388-7260-4449-b8b9-67d9719837](https://www.google.com/search?q=https://github.com/user-attachments/assets/679be388-7260-4449-b8b9-67d9719837)" alt="RobotFinalAssembly1" height="500"\>

## ✨ Key Features

  * [cite\_start]**Dynamic Self-Balancing**: Implements a robust **Multi-rate PID control** algorithm to ensure stable balancing and precise locomotion[cite: 22]. [cite\_start]The pitch control loop uses an **Adaptive Derivative Gain** to achieve high stability while eliminating jitter[cite: 758, 765].
  * [cite\_start]**Advanced Sensor Fusion**: Utilizes an **Extended Kalman Filter (EKF)** for accurate state estimation[cite: 95]. [cite\_start]It fuses data from the **MPU-6050** IMU (attitude) [cite: 262] [cite\_start]and **Hall-effect motor encoders** (odometry) [cite: 21, 828] to get a reliable estimate of the robot's pitch angle and position.
  * [cite\_start]**Real-time SLAM**: The host PC application integrates robot odometry with the **TeraRanger Multiflex** ToF sensor readings [cite: 19] to perform SLAM, building a 2D point cloud map of the environment on the fly.
  * [cite\_start]**Live Data Visualization**: A custom **PyQtGraph** application provides a rich, interactive 2D plot displaying the robot's estimated position, historical path, and the mapped environmental points[cite: 1089].
  * [cite\_start]**Wi-Fi Communication Hub**: The ESP32-S3 acts as a **Soft Access Point (SoftAP)**[cite: 1081], creating its own Wi-Fi network. [cite\_start]It streams sensor telemetry and receives control commands over a low-latency **TCP socket** connection[cite: 1085, 1124].
  * **Autonomous & Manual Control**: The robot can be driven manually via keyboard commands or operate autonomously, using the generated map and a **wall-following algorithm** to solve mazes.

-----

## 🛠️ System Architecture & Technologies

The system is split into two main parts: the on-board embedded controllers and the host PC application.

### 1\. Robot Hardware & Firmware (C++ / PlatformIO)

| Component | Role | Details |
| :--- | :--- | :--- |
| **Mobile Base** | **Arduino Nano** | [cite\_start]The low-level controller responsible for real-time balancing and motion[cite: 21]. [cite\_start]It runs the high-frequency, multi-rate PID control loops[cite: 22]. |
| **Comm. Hub** | **ESP32-S3-DevKitC-1** | [cite\_start]The high-level controller and main hub[cite: 23]. [cite\_start]It aggregates data from the Arduino Nano and ToF sensors [cite: 23][cite\_start], and manages the Wi-Fi (SoftAP) TCP connection to the PC[cite: 1081, 1085]. |
| **Attitude Sensor** | **MPU-6050 IMU** | [cite\_start]Provides 6-axis accelerometer and gyroscope data for orientation and pitch angle estimation[cite: 262, 327]. |
| **Mapping Sensor** | **TeraRanger Multiflex** | [cite\_start]An array of 8 ToF (Time-of-Flight) sensors used for 2D environmental mapping[cite: 19, 487]. |
| **Odometry Sensors** | **Hall-Effect Encoders** | [cite\_start]Integrated into the DC motors [cite: 455][cite\_start], they provide precise odometry (distance and speed) measurements[cite: 21, 828]. |

### 2\. Host PC Application (Python)

  * **Logic**: **Python** for the main application, SLAM algorithms, and maze-solving logic.
  * **GUI**: **PyQt5** for the application window and user controls.
  * **Visualization**: **PyQtGraph** for high-performance, real-time 2D plotting of the map and robot state.
  * **Data Processing**: **NumPy** for efficient numerical operations.

-----

## 🎥 Demo

[Link to Demo Video](https://github.com/user-attachments/assets/0ca18e00-be7d-4444-9bbe-24fa2c01fbb0)

-----

## 📂 Project Structure

```
.
[cite_start]├── communication_hub_code/  # Firmware for the ESP32-S3 (Wi-Fi, Sensor Hub) [cite: 23]
[cite_start]├── mobile_base_code/        # PlatformIO project for the Arduino Nano (Balancing, Motor Control) [cite: 21]
├── tests/                   # Test scripts and code
├── videos/                  # Demo videos
├── .gitignore
├── LICENSE
├── logger_config.py         # Logging configuration
├── main.py                  # Main Python application entry point (SLAM & Visualization)
├── mazeSolver.py            # Autonomous maze solving logic
├── README.md                # This file
├── requirements.txt         # Python dependencies
└── robotInterface.py        # Interface for robot (TCP) communication and control
```

-----

## 🚀 Installation & Setup

### 1\. Clone the Repository

```bash
git clone https://github.com/haris-mujeeb/Maze_Solver.git
cd Maze_Solver
```

### 2\. Install Python Dependencies

```bash
pip install -r requirements.txt
```

### 3\. Robot Firmware Setup (Crucial)

[cite\_start]You will need **[PlatformIO](https://platformio.org/)** (ideally as a VS Code extension) to build and upload the firmware for both microcontrollers[cite: 25].

#### A. Mobile Base (Arduino Nano)

This firmware handles the core self-balancing and motor control.

1.  Open the `mobile_base_code/` directory in VS Code with PlatformIO.
2.  [cite\_start]Connect your **Arduino Nano**[cite: 21].
3.  Build and upload the project.

#### B. Communication Hub (ESP32-S3)

This firmware manages sensor data aggregation and Wi-Fi communication.

1.  Open the `communication_hub_code/` directory in VS Code with PlatformIO.
2.  [cite\_start]Connect your **ESP32-S3-DevKitC-1**[cite: 23].
3.  Build and upload the project.
4.  [cite\_start]This firmware will create a Wi-Fi Access Point (SoftAP)[cite: 1081]. The default IP address is typically `192.168.4.1`.

-----

## ⚙️ Configuration

The network settings for Wi-Fi communication are defined in `main.py`. These must match your robot's network configuration.

```python
# main.py
RECEIVE_HOST = '0.0.0.0'  # PC: Listen on all available interfaces for incoming robot data
[cite_start]RECEIVE_PORT = 12346      # PC: Port for receiving telemetry data (via TCP) [cite: 1085]
[cite_start]SEND_HOST = '192.168.4.1' # ROBOT'S IP: Default IP for ESP32-S3 SoftAP mode [cite: 1081]
[cite_start]SEND_PORT = 12345         # ROBOT'S PORT: Port for sending commands (via TCP) [cite: 1124]
```

  * **`SEND_HOST`**: This **must be the IP address of your ESP32-S3**. [cite\_start]If you are connecting your PC directly to the robot's Wi-Fi network (SoftAP mode), this will be `192.168.4.1`[cite: 1081].
  * Ensure the ports match those configured in the ESP32-S3 firmware.

-----

## 🕹️ Usage

1.  **Power on the robot**. The Arduino Nano will begin balancing, and the ESP32-S3 will start its Wi-Fi Access Point.
2.  **Connect your PC** to the Wi-Fi network created by the ESP32-S3.
3.  **Run the main application** on your host PC:
    ```bash
    python main.py
    ```
4.  **Control the robot:**
      * The PyQtGraph application will launch, showing the live SLAM visualization.
      * Use the following keyboard commands for manual control:
          * **W**: Move forward
          * **S**: Move backward
          * **A**: Turn left
          * **D**: Turn right
          * **R**: Stop
          * **G**: Go to a manually entered target (prompts for X, Y coordinates)
      * You can also **left-click on the plot** to set a target position for the robot to navigate to autonomously.

## Contributing

Contributions are welcome\! Please feel free to submit a pull request or open an issue if you have any suggestions or find any bugs.

## License

This project is licensed under the MIT License. See the [LICENSE](https://www.google.com/search?q=LICENSE) file for details.