import socket
import time

# Configuration details for the ESP32
host = '192.168.4.1'  # ESP32 Access Point IP
port = 12345           # Server port (same as in ESP32 code)

# Function to send data to ESP32
def send_data_to_wifi(data):
    # Create a socket to connect to the ESP32
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            client_socket.connect((host, port))  # Connect to the ESP32 server
            print(f"Connected to ESP32 at {host}:{port}")
            
            # Send the data to ESP32
            client_socket.sendall(data.encode())
            print(f"Sent to ESP32: {data}")
            
            # # Receive acknowledgment from ESP32
            # response = client_socket.recv(1024).decode()  # Expected to be "OK\n"
            # print(f"Received acknowledgment from ESP32: {response.strip()}")
            
            # client_socket.close()
        
        except ConnectionError as e:
            print(f"Connection failed: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")

# Example of how to use the function
if __name__ == "__main__":
    try:
        while True:
            # Create some data to send (can be changed to dynamic values)
            data_to_send = "Test data from PC to ESP32"
            send_data_to_wifi(data_to_send)
            
            # Wait for a few seconds before sending again
            time.sleep(0.1)  # Send every 5 seconds
            
    except KeyboardInterrupt:
        print("Program interrupted by user.")
