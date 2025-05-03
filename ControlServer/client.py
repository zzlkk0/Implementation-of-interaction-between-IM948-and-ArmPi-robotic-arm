import asyncio
import json
import logging
import zmq
import zmq.asyncio
from typing import Optional, Dict, Any, Callable
import yaml

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("TransmissionClient")

class TransmissionClient:
    def __init__(self, config_path: Optional[str] = None):
        self.config = self._load_config(config_path)
        self.zmq_context = zmq.asyncio.Context()
        self.zmq_socket = None
        self.message_handlers: Dict[str, Callable] = {}
        
    def _load_config(self, path: Optional[str]) -> dict:
        if path:
            with open(path) as f:
                return yaml.safe_load(f)
        # Added a new config key for message preview length (if needed on the client side)
        return {
            'router_host': '127.0.0.1',
            'zmq_port': 5555,
            'message_preview_length': 10
        }
        
    async def connect(self):
        """Establish connection to the ZMQ router"""
        self.zmq_socket = self.zmq_context.socket(zmq.DEALER)
        self.zmq_socket.connect(
            f"tcp://{self.config['router_host']}:{self.config['zmq_port']}"
        )
        logger.info("Connected to ZMQ router")
        asyncio.create_task(self._handle_messages())
        
    async def _handle_messages(self):
        """Background task to handle incoming ZMQ messages"""
        while True:
            try:
                message = await self.zmq_socket.recv_multipart()
                topic = message[0].decode()
                payload = json.loads(message[1].decode())
                if topic in self.message_handlers:
                    await self.message_handlers[topic](payload)
                else:
                    logger.warning(f"No handler for topic: {topic}")
            except Exception as e:
                logger.error(f"Error handling message: {e}")
            await asyncio.sleep(0.1)
            
    async def send_message(self, topic: str, payload: dict):
        """Send a message through ZMQ"""
        if not self.zmq_socket:
            raise RuntimeError("Client not connected")
        
        message = [
            topic.encode(),
            json.dumps(payload).encode()
        ]
        await self.zmq_socket.send_multipart(message)
        
    async def create_stream(self, stream_id: str, camera_url: Optional[str] = None):
        """
        This method used to create a WebRTC stream.
        WebRTC is now removed; this function is kept for compatibility and simply logs a message.
        """
        logger.info("WebRTC is disabled. 'create_stream' is a no-op in ZMQ-only mode.")
        return None
        
    def on_message(self, topic: str):
        """Decorator for registering message handlers"""
        def decorator(func: Callable):
            self.message_handlers[topic] = func
            return func
        return decorator
        
    async def close(self):
        """Clean up ZMQ resources"""
        if self.zmq_socket:
            self.zmq_socket.close()
        self.zmq_context.term()
