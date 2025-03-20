import matplotlib.pyplot as plt
import matplotlib.animation as animation
from robot_interface import RobotInterface
import numpy as np
import time

robot = RobotInterface("COM11", 9600)
robot.set_logging(True) # enable logging
robot.start_reading()

# Plot Setup
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
scat = ax.scatter([], [])
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.set_title("Live ToF Sensor Data (Polar)")
R_MAX = 400  # Fixed radial length

def update(frame):
    tof_data = robot.tof_sensor_values
    if tof_data:
        num_sensors = len(tof_data)
        angles = np.linspace(0, 2 * np.pi, num_sensors, endpoint=False)
        adjusted_data = [R_MAX if val == -1 else val for val in tof_data]
        ax.clear()
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(1)
        ax.set_rlim(0, R_MAX)
        ax.set_title("Live ToF Sensor Data (Polar)")
        scat = ax.scatter(angles, adjusted_data)
    return scat,

ani = animation.FuncAnimation(fig, update, blit=False, interval=100)  # Adjust interval as needed


try:
  last_toggle = time.time()
  interval = 1
  command = "MOVE"
  speed = 25
  move_direction = 1
  
  plt.show()
  
  while True:
    value = 10 * move_direction
    
    formatted_command = f"{command},{value},{speed}\n"
    
    if time.time() - last_toggle >= interval:
      move_direction *= -1
      last_toggle = time.time() 
    
    robot.send_command(formatted_command)
    time.sleep(0.5)

except KeyboardInterrupt:
    print("Program interrupted.")
finally:
    robot.close()