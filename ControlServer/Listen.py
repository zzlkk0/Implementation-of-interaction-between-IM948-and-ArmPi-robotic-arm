import asyncio
from Board import getBusServoPulse
from client import TransmissionClient
from loguru import logger

# Global flag to indicate if we should process REMOTE_VALUE messages
active = False

async def handle_remote_control(payload):
    global active
    # Assuming the payload is a simple string like "ON" or "OFF"
    if payload == "ON":
        active = True
        print("REMOTE_CONTROL: ON - Now listening to REMOTE_VALUE")
    elif payload == "OFF":
        active = False
        print("REMOTE_CONTROL: OFF - Ignoring REMOTE_VALUE messages")
    else:
        print(f"REMOTE_CONTROL: Unrecognized payload: {payload}")

async def handle_remote_value(payload):
    if active:
        print("Received REMOTE_VALUE:", payload)
    # If active is False, we do nothing

async def send_servo_pulses():
    client = TransmissionClient(config_path='config.yaml')
    await client.connect()
    
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
    finally:
        await client.close()

async def main():
    # Initialize the client from client.py
    client = TransmissionClient()
    
    # Register message handlers
    client.on_message("REMOTE_CONTROL")(handle_remote_control)
    client.on_message("REMOTE_VALUE")(handle_remote_value)
    
    # Connect to the ZMQ router
    await client.connect()
    
    # Keep the program running indefinitely to continue listening for messages.
    while True:
        await asyncio.sleep(1)

#if __name__ == "__main__":
#    asyncio.run(main())
#    asyncio.run(send_servo_pulse())
loop = asyncio.get_event_loop()
task1_future = asyncio.ensure_future(main())
task2_future = asyncio.ensure_future(send_servo_pulses())

# Run both tasks
loop.run_until_complete(asyncio.gather(task1_future, task2_future))
loop.close()


