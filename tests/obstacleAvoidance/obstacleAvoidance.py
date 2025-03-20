import serial
import time

# Serial port configuration
SERIAL_PORT = "COM6"
BAUD_RATE = 9600
TIMEOUT = 1  # seconds

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

def sendCommands(data_to_send):
  # Convert the string to bytes (ASCII encoding)
  data_bytes = data_to_send.encode('ascii')
  # Send the data over the serial port
  ser.write(data_bytes)
  print(f"Sent: {data_to_send}")

def readTele():
  dataTele = ser.read_until("\n")
  print(dataTele)
try:
  # Check if the serial port is open
  if ser.is_open:
    print(f"Serial port {SERIAL_PORT} opened successfully.")

  # ASCII data to send
  time.sleep(2)
  moveForw = "1,20,100"
  sendCommands(moveForw)

  # ASCII data to send
  time.sleep(2)
  moveBack = "2,-90,100"
  sendCommands(moveBack)

  # ASCII data to send
  time.sleep(2)
  moveForw = "1,40,100"
  sendCommands(moveForw)

  # ASCII data to send
  time.sleep(2)
  moveBack = "2,-180,100"
  sendCommands(moveBack) 

  # ASCII data to send
  time.sleep(2)
  moveForw = "1,60,100"
  sendCommands(moveForw) 

  # ASCII data to send
  time.sleep(2)
  moveBack = "2,-270,100"
  sendCommands(moveBack) 

  # ASCII data to send
  time.sleep(2)
  moveForw = "1,80,100"
  sendCommands(moveForw)

  # ASCII data to send
  time.sleep(2)
  moveBack = "2,0,100"
  sendCommands(moveBack)

except Exception as e:
  print(f"An error occurred: {e}")

finally:
  # Close the serial port
  ser.close()
  print("Serial port closed.")

# Close serial port
ser.close()