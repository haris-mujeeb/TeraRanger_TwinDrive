import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


# Configure the serial connection
COM_PORT = 'COM3'  # Change this to your Arduino's COM port
BAUD_RATE = 115200    # Make sure this matches the Arduino's baud rate

# Create a serial connection
ser = serial.Serial(COM_PORT, BAUD_RATE)

# Initialize the robot's position and path
position = [0, 0]  # Starting at the origin (0, 0)
path_x = [position[0]]
path_y = [position[1]]

# Create the figure and axis for the plot
# fig, ax = plt.subplots()
# line, = ax.plot([], [], marker='o', linestyle='-', color='b')
# ax.set_xlim(-150, 150)  # Adjust limits as needed
# ax.set_ylim(-10, 10)    # Adjust limits as needed
# ax.axhline(0, color='black', linewidth=0.5, ls='--')
# ax.axvline(0, color='black', linewidth=0.5, ls='--')
# ax.grid(color='gray', linestyle='--', linewidth=0.5)
# ax.set_title("Live Robot Path Visualization")
# ax.set_xlabel("X Position (cm)")
# ax.set_ylabel("Y Position (cm)")

# def update(frame):
#     global position
#     # Read the command from the Arduino
#     if ser.in_waiting > 0:
#         packet = ser.readline().decode('utf-8').strip()
#         command, value = packet.split(',')
#         command = command.strip()
#         value = float(value.strip())

#         # Update position based on command
#         if command == 'M':
#             position[0] += value  # Move along the x-axis
#             path_x.append(position[0])
#             path_y.append(position[1])

#     # Update the line data
#     line.set_data(path_x, path_y)
#     return line,

# # Create an animation
# ani = animation.FuncAnimation(fig, update, frames=np.arange(0, 100), blit=True, interval=100)

# plt.show()

def send_command(command):
    packet = f"{command}\n"
    ser.write(packet.encode())  # Send the command to the Arduino
    print(f"Sent: {packet.strip()}")  # Print the sent command for confirmation


try:
    time.sleep(5)  # Wait for 1 second for robot to start

    # Move forward 
    send_command('Move,100')  # Move forward 100 cm
    time.sleep(2)  # Wait for 1 second

    # Move backward
    send_command('Move,-100')  # Move backward 100 cm
    time.sleep(2)  # Wait for 1 second

    # Stop the robot
    send_command('Move,0')  # Stop the robot

    # Rotate clockwise (90 degrees)
    send_command('Turn,90')  
    time.sleep(2)  # Wait for 1 second
    
    # Rotate counterclockwise (90 degrees)
    send_command('Turn,-90') 
    time.sleep(2)  # Wait for 1 second

    # Rotate to 0 degrees
    send_command('Turn,0')  # Stop the robot
    time.sleep(2)  # Wait for 1 second

except Exception as e:
    print(f"Error occurred while sending command to Arduino:\n{str(e)}")  # Print error if any occurs during transmission

finally:
    # Close the serial connection
    ser.close()

