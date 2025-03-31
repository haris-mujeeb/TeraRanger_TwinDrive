# robot_interface.py
import asyncio
from communication import send_command, receive_data, connect_to_robot  # Import the communication functions


class RobotInterface:
    def __init__(self, ip, port, reconnect_delay=3):
        self.ip = ip
        self.port = port
        self.reconnect_delay = reconnect_delay
        self.reader = None
        self.writer = None

    async def _connect(self):
        """Private method to handle the connection to the robot."""
        while True:
            try:
                self.reader, self.writer = await connect_to_robot(self.ip, self.port)
                print(f"✅ Connected to Robot at {self.ip}:{self.port}")
                break  # Break the loop once connected
            except Exception as e:
                print(f"⚠️ Connection failed: {e}. Retrying in {self.reconnect_delay} seconds...")
                await asyncio.sleep(self.reconnect_delay)

    async def send_command(self, cmd):
        """Send a command to the robot."""
        if self.writer is None or self.writer.is_closing():
            await self._connect()  # Ensure we're connected before sending the command

        # Send the command
        success = await send_command(self.writer, cmd)
        if not success:
            print("⚠️ Command failed.")
        return success

    async def receive_data(self):
        """Receive data from the robot."""
        if self.reader is None:
            await self._connect()  # Ensure we're connected before receiving data

        # Receive the data
        data = await receive_data(self.reader)
        return data

    async def reconnect_if_needed(self):
        """Check if the connection is lost and attempt to reconnect."""
        if self.reader is None or self.writer is None or self.writer.is_closing():
            print("🔄 Attempting to reconnect...")
            await self._connect()

# Wrapper functions for user convenience
async def send_command_to_robot(robot_interface, cmd):
    return await robot_interface.send_command(cmd)

async def receive_data_from_robot(robot_interface):
    return await robot_interface.receive_data()
