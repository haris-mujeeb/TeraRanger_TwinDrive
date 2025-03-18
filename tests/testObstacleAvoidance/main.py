# main.py
import asyncio
from robot_interface import RobotInterface, send_command_to_robot, receive_data_from_robot

async def main():
    """Main control loop for communication."""
    robot = RobotInterface("192.168.136.112", 12345)
    
    while True:
        try:
            # Ensure the connection is alive, reconnect if needed
            await robot.reconnect_if_needed()

            # Send a movement command to the robot
            cmd = "MOVE 10"
            success = await send_command_to_robot(robot, cmd)
            if success:
                print(f"📤 Sent command: {cmd}")

            # Receive data from the robot
            data = await receive_data_from_robot(robot)
            if data:
                print(f"📥 Received: {data}")

            await asyncio.sleep(1)  # Adjust delay as needed

        except (ConnectionError, OSError) as e:
            print(f"⚠️ Connection lost: {e}. Reconnecting in {robot.reconnect_delay} seconds...")
            await asyncio.sleep(robot.reconnect_delay)

# Run the event loop
asyncio.run(main())
