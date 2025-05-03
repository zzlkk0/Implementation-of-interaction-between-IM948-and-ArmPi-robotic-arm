import asyncio
from Board import getBusServoPulse
from client import TransmissionClient
from loguru import logger


#
PulseID_A2=[[1000,24,501,59,775,588,501],[1000,317,501,123,799,493,501],[1000,420,501,137,796,447,500],[1000,420,501,123,799,493,501],[1000,420,501,59,775,588,501]]


# Global flag to indicate if we should process REMOTE_VALUE messages
active = False

async def handle_remote_control(payload):
    global active
    # Check if payload is a dictionary and extract the command from the 'message' key.
    command = payload.get("message") if isinstance(payload, dict) else payload
    if command == "ON":
        active = True
        print("REMOTE_CONTROL: ON - Now listening to REMOTE_VALUE")
    elif command == "OFF":
        active = False
        print("REMOTE_CONTROL: OFF - Ignoring REMOTE_VALUE messages")
    else:
        print(f"REMOTE_CONTROL: Unrecognized command: {command}")

async def handle_remote_value(payload):
    if active:
        print("Received REMOTE_VALUE:", payload)
    # If active is False, do nothing

async def send_servo_pulses(client):
    try:
        while True:
            # Retrieve servo pulses for servo IDs 1 through 6
            servo_values = {}
            for servo_id in range(1, 7):
                pulse = getBusServoPulse(servo_id)
                servo_values[servo_id] = pulse
                logger.info(f"Servo {servo_id} pulse: {pulse}")
            
            # Create a payload containing all servo values
            payload = {"servo_values": servo_values}
            await client.send_message("servo_pulses", payload)
            
            # Wait before sending the next update (adjust interval as necessary)
            await asyncio.sleep(3)
    except asyncio.CancelledError:
        logger.info("Servo pulse sending cancelled.")

async def main():
    # Create a single client instance
    client = TransmissionClient(config_path='config.yaml')
    
    # Register message handlers
    client.on_message("REMOTE_CONTROL")(handle_remote_control)
    client.on_message("REMOTE_VALUE")(handle_remote_value)
    
    # Connect the client
    await client.connect()
    
    # Run the servo pulse sending task concurrently
    task_servo = asyncio.create_task(send_servo_pulses(client))
    
    # Keep the program running indefinitely to continue listening for messages.
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("Program interrupted. Shutting down...")
    finally:
        task_servo.cancel()
        await client.close()

if __name__ == "__main__":
    asyncio.run(main())

