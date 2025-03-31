import socket

# Configuration details for the ESP32
host = '192.168.4.2'  # Python server IP
port = 12345           # Server port

# Function to receive data from ESP32 and return it as a string
def get_data_from_wifi():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    
    print(f"Listening on {host}:{port}...")
  
    client_socket, client_address = server_socket.accept()
    print(f"Connection established with {client_address}")
  
    try:
        data = ""
        while True:
            received_data = client_socket.recv(1024).decode()  # Read data from ESP32
            
            if received_data:
                print(f"Received from ESP32: {received_data.strip()}")
                client_socket.sendall(b"OK\n")  # Send acknowledgment
                data += received_data.strip()  # Add the received data to our data string
            else:
                print("No data received. Connection closed.")
                break  # Break out of the loop if no data is received (client disconnected)
        
        return data  # Return the accumulated data from the ESP32
    except ConnectionResetError:
        print("Connection reset by ESP32.")
    finally:
        client_socket.close()
        print("Connection closed.")

# Example usage of the function
if __name__ == "__main__":
  while True:
    try:
      data_received = get_data_from_wifi()
      print(f"Data received: {data_received}")
    except KeyboardInterrupt:
      break
      pass