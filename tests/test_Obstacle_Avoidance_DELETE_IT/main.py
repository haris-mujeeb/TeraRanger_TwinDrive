from robot_interface import RobotInterface
from obstacle_avoider import ObstacleAvoider
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

matplotlib.use('TkAgg')  # Force the correct backend.

# Configuration Variables
SERIAL_PORT = "COM23"  # Serial port for the robot connection
SERIAL_BAUD_RATE = 9600  # Baud rate for serial communication
ROBOT_INITIALIZATION_DELAY = 0.5  # Time to wait for the robot to initialize (in seconds)
OBSTACLE_AVOIDANCE_TARGET_DISTANCE = -50  # Target distance for obstacle avoidance

def setup_robot():
    robot = RobotInterface(SERIAL_PORT, SERIAL_BAUD_RATE)
    robot.set_logging(True)  # Enable logging
    time.sleep(ROBOT_INITIALIZATION_DELAY)  # Allow time for the robot to initialize
    return robot

def setup_plot(robot):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.clear()
    ax.set_theta_zero_location("N")
    ax.set_rlim(0, robot.R_MAX)  # Set the radial limit based on the maximum range
    ax.set_theta_direction(1)
    ax.set_title("Live ToF Sensor Data (Polar)")
    return fig, ax

def update_plot(frame, robot, ax):
    if robot.tof_sensor_values:
        num_sensors = len(robot.tof_sensor_values)
        angles = np.linspace(np.pi / 8, (2 * np.pi) + (np.pi / 8), num_sensors, endpoint=False)
        adjusted_data = [robot.R_MAX if val == -1 else val for val in robot.tof_sensor_values]
        
        ax.clear()
        ax.set_theta_zero_location("N")
        # ax.set_theta_direction(1)
        # ax.set_rlim(0, robot.R_MAX)  # Set the radial limit based on the maximum range
        # ax.set_title("Live ToF Sensor Data (Polar)")
        ax.scatter(angles, adjusted_data)
    return ax,

def main():
    robot = setup_robot()
    avoider = ObstacleAvoider(robot)
    fig, ax = setup_plot(robot)
    
    ani = animation.FuncAnimation(fig, update_plot, fargs=(robot, ax), blit=False, interval=100)
    
    try:
        plt.show()  # Show the plot after the main loop.
        while True:
            time.sleep(0.5)
            avoider.abs_target_distance = OBSTACLE_AVOIDANCE_TARGET_DISTANCE  # Set the target distance for obstacle avoidance
            avoider._next_point()
            avoider._centering()
    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        robot.close()

if __name__ == "__main__":
    main()
