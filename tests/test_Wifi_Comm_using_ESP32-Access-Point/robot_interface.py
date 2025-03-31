import socket
import threading
import queue
import logging
import logger_config

# Set up logging configuration
logger_config.setup_logging()

class RobotInterface:
    def __init__(self, host_receive, port_receive, host_send, port_send):
        self.host_receive = host_receive
        self.port_receive = port_receive
        self.host_send = host_send
        self.port_send = port_send
        self.data_queue = queue.Queue()
        threading.Thread(target=self._get_data_from_wifi_loop, daemon=True).start()
        
        self.logger = logging.getLogger(__name__)
        self.set_logging(False)
    
    def set_logging(self, enabled: bool):
      """Enable or disable logging."""
      self.logger.setLevel(logging.INFO if enabled else logging.WARNING)
      self.logger.info("Logging is %s.", "enabled" if enabled else "disabled")
  
    def _get_data_from_wifi_loop(self):
        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
                    server_socket.bind((self.host_receive, self.port_receive))
                    server_socket.listen(1)
                    self.logger.info(f"👂 Listening on {self.host_receive}:{self.port_receive}...")

                    client_socket, client_address = server_socket.accept()
                    self.logger.info(f"🤝 Connection from {client_address}")

                    while True:
                        received_data = client_socket.recv(1024).decode().strip()
                        if received_data:
                            self.logger.info(f"📥 Received: {received_data}")
                            client_socket.sendall(b"OK\n")
                            self.data_queue.put(received_data)
                        else:
                            self.logger.info("🚪 Connection closed.")
                            break
                          

            except ConnectionResetError:
                self.logger.warning("💥 Connection reset.")
            except KeyboardInterrupt:
                self.logger.error("🛑 Program interrupted by user.")
            except Exception as e:
                self.logger.error(f"🚨 Error: {e}")
                self.logger.exception("Exception occurred") #for full stack trace.

    def get_data_from_wifi(self):
        """Returns the most recent data from the queue, or an empty string."""
        data = ""
        try:
            while not self.data_queue.empty():
                data += self.data_queue.get_nowait()
            return data
        except queue.Empty:
            return ""

    def send_data_to_wifi(self, data):
        """Sends data to the ESP32."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.connect((self.host_send, self.port_send))
                self.logger.info(f"🚀 Connected to {self.host_send}:{self.port_send}")
                client_socket.sendall(data.encode())
                self.logger.info(f"📤 Sent: {data}")
        except Exception as e:
            self.logger.error(f"❌ Send error: {e}")
            self.logger.exception("Exception occurred") #for full stack trace.