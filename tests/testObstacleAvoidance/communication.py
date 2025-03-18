# communication.py
import asyncio

BUFFER_SIZE = 1024
DATA_TIMEOUT = 5  # Timeout for receiving data

async def connect_to_robot(ip, port):
    """Attempts to connect to the ESP32 server asynchronously."""
    while True:
        try:
            reader, writer = await asyncio.open_connection(ip, port)
            print(f"✅ Connected to Robot at {ip}:{port}")
            return reader, writer
        except Exception as e:
            print(f"⚠️ Connection failed: {e}. Retrying...")
            await asyncio.sleep(3)

async def send_command(writer, command):
    """Send a command to the ESP32 robot."""
    try:
        writer.write(command.encode() + b"\n")
        await writer.drain()
        print(f"📤 Sent command: {command}")
        return True
    except Exception as e:
        print(f"⚠️ Send error: {e}")
        return False

async def receive_data(reader):
    """Receive data from the ESP32 asynchronously."""
    try:
        data = await asyncio.wait_for(reader.read(BUFFER_SIZE), timeout=DATA_TIMEOUT)
        return data.decode('utf-8').strip() if data else None
    except asyncio.TimeoutError:
        print("⚠️ Receive timeout: No response from Robot.")
        return None
    except Exception as e:
        print(f"⚠️ Receive error: {e}")
        return None
