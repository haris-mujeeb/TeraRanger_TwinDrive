# TeraRanger Twin Drive
 A robot featuring TeraRanger Multiflex. It consist of a [TeraRanger-Multiflex ToF-sensor array](https://github.com/haris-mujeeb/TeraRanger-Multiflex-DEMO) mounted on top of a [Two-wheeled Self Balancing Robot](https://github.com/haris-mujeeb/Self-Balancing-Robot) as the mobile base.


<img src="https://github.com/user-attachments/assets/679be388-7260-4449-b8b9-67d971989837" alt="RobotFinalAssembly1" height="500">


# Features:
- **Real-time SLAM**: Integrates robot odometry (gyro, distance) with ToF sensor readings to build and update an environmental map on the fly.
- **Live Data Visualization**: A custom PyQtGraph application provides a rich, interactive 2D plot displaying:
 - The robot's current position and orientation.
 - The robot's historical path.
 - Mapped environmental points detected by the ToF sensors. 
- **Wi-Fi Communication**: Establishes a robust Wi-Fi connection for seamless data transmission between the robot and a host PC.
- **Keyboard Control**: Intuitive keyboard commands allow for manual control of the robot's movement (forward, backward, turn left, turn right, stop).

# Demo:


https://github.com/user-attachments/assets/0ca18e00-be7d-4444-9bbe-24fa2c01fbb0



# Installation:
 - **Clone the repository**:
``git clone https://github.com/your-username/Maze-Solver-Robot-SLAM.git
cd Maze-Solver-Robot-SLAM
``

- **Install Python dependencies**:
`` 
pip install numpy pyqtgraph PyQt5
``

- **Robot Setup**: 
Ensure your robot's firmware is configured to:
 - Send sensor data (ToF, gyro, odometry) over Wi-Fi (TCP/IP) to the host_receive address and port_receive.
 - Listen for movement commands on host_send and port_send.
 - Detailed robot-side code (e.g., Arduino/ESP32) is included in there respective repositories (e.g. [TeraRanger-Multiflex ToF-sensor array](https://github.com/haris-mujeeb/TeraRanger-Multiflex-DEMO) and [Two-wheeled Self Balancing Robot](https://github.com/haris-mujeeb/Self-Balancing-Robot))
