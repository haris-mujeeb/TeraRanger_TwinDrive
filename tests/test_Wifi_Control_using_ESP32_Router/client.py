import socket
import time

# Configuration
ROBOT_IP = "192.168.160.83"  # Replace with your ESP32 IP address
ROBOT_PORT = 12345  # Same port as ESP32 server

BUFFER_SIZE = 1024  # Max data to receive from ESP32
RECEIVE_TIMEOUT = 1 # Timeout for receiving data (in seconds)
RECONNECT_DELAY = 5 # delay in seconds before reconnecting

def connect_to_robot():
  """Connects to the Robot and returns the socket object."""
  try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ROBOT_IP, ROBOT_PORT))
    client_socket.settimeout(RECEIVE_TIMEOUT)
    print(f"Connected to Robot at {ROBOT_IP}:{ROBOT_PORT}")
    return client_socket
  except Exception as e:
    print(f"Error connecting to Robot: {e}")
    return None
      
def send_command(client_socket, command):
  if client_socket:  
    """Send a command to the Robot without delay."""
    try:
        client_socket.sendall((command + "\n").encode())
        print(f"Sent command: {command}")
        return True
    except Exception as e:
        print(f"Error sending command: {e}")
        return True
  return False

def receive_data(client_socket):
  if client_socket:    
    """Receive data from the Robot with a timeout."""
    try:
        data = client_socket.recv(BUFFER_SIZE)
        if data:
          print(f"Received data: {data}")
        return data.decode('utf-8').strip()
    except socket.timeout:
        print("Timeout: No data received from Robot.")
        return None
    except Exception as e:
        print(f"Error receiving data: {e}")
        return None
  return None

def main():
  client_socket = connect_to_robot()
  if client_socket is None:
    print("Failed to establish initial connection. Retrying...")
    return
  
  try:
    while True:
      # Request data immediately
      if not send_command(client_socket, "GET_DATA"):
        raise Exception("Send failed, forcing reconnect") #Force reconnect
      
      telemetry = receive_data(client_socket)
      # if telemetry:
      #   if telemetry.startswith("ERROR"):
      #     print(f"Received data from Robot: {telemetry}")
      #   else:
      #     print(f"Received data from Robot: {telemetry}")

      # Send a command without delay (Example: Move 100 units)
      send_command(client_socket, "MOVE 100")  # Fixed format with space, not comma
      print("Sent command to move 100 units.")

      # Small delay to allow the ESP32 to process the command
      time.sleep(0.001)

  except KeyboardInterrupt:
        print("Server stopped.")
  except Exception as e:
      print(f"Unexpected error: {e}")
  finally:
      if client_socket:
          try:
            client_socket.close()
          except Exception as e:
              print(f"Error closing socket: {e}")

  print(f"Attempting reconnection in {RECONNECT_DELAY} seconds...")
  time.sleep(RECONNECT_DELAY)
  main()
  return
  
if __name__ == "__main__":
  main()