import time
from robot_interface import RobotInterface
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

# Configuration details for the ESP32
host_receive = '192.168.4.2'  # Python server IP
port_receive = 12345           # Server port for receiving data
host_send = '192.168.4.1'      # ESP32 Access Point IP
port_send = 12345               # Server port for sending data


def setup_robot():
  robot = RobotInterface(host_receive, port_receive, host_send, port_send)
  robot.set_logging(True) 
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
        
        ax.clear()
        ax.set_theta_zero_location("N")
        # ax.set_theta_direction(1)
        ax.set_rlim(0, robot.R_MAX)  # Set the radial limit based on the maximum range
        # ax.set_title("Live ToF Sensor Data (Polar)")
        ax.scatter(angles, robot.tof_sensor_values)
    return ax,


def right_wall_follower_thread(robot):
    """Run the obstacle avoider in a separate thread."""
    while True:
        try:
            robot.right_wall_follower()
            time.sleep(0.1)  # Adjust the sleep time as needed
        except Exception as e:
            print(f" 🛑 Error in obstacle avoider: {e}.")
            break


def move_test_thread(robot):
  """Run the move test in a separate thread."""
  while not robot.enter_the_maze():
    time.sleep(0.2)
    pass
  
  print(f"Entered the Maze !!!")
  
  while True:
    try:
      robot.move_relative(20)
      time.sleep(5)  # Adjust the sleep time as needed
      robot.move_relative(-20)
      time.sleep(5)  # Adjust the sleep time as needed
    except Exception as e:
      print(f" 🛑 Error in Move: {e}.")
      break


def main():
  robot = setup_robot()
  fig, ax = setup_plot(robot)
  ani = animation.FuncAnimation(fig, update_plot, fargs=(robot, ax), blit=False, interval=100)

  # time.sleep(4)  # Adjust the sleep time as needed

  # move_thread = threading.Thread(target=move_test_thread, args=(robot,))
  # move_thread.daemon = True
  # move_thread.start()

  # time.sleep(2)  # Adjust the sleep time as needed

  # # Start the obstacle avoider in a new thread
  # wall_follower_thread = threading.Thread(target=right_wall_follower_thread, args=(robot,))
  # wall_follower_thread.daemon = True  # Daemonize thread to exit when main program exits
  # wall_follower_thread.start()
  
  # time.sleep(2)  # Adjust the sleep time as needed

  plt.show()  # Show the plot after the main loop.

if __name__ == "__main__":
  main()