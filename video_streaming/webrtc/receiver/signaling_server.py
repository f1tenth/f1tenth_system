#!/usr/bin/env python3
import asyncio
import json
import logging
import sys
from typing import Dict

try:
    import websockets
except ImportError:
    print("Error: websockets library not found. Install with: pip install websockets")
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

class SignalingRoom:
    def __init__(self, room_id: str):
        self.room_id = room_id
        self.sender = None
        self.receiver = None
    
    def is_full(self) -> bool:
        return self.sender is not None and self.receiver is not None
    
    def is_empty(self) -> bool:
        return self.sender is None and self.receiver is None

class SignalingServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 8765):
        self.host = host
        self.port = port
        self.rooms: Dict[str, SignalingRoom] = {}
        self.connections: Dict = {}
        
    async def safe_send(self, websocket, message: str) -> bool:
        """Safely send a message. If the TCP connection is dead, catch the error."""
        if websocket is None:
            return False
        try:
            await websocket.send(message)
            return True
        except Exception as e:
            logger.warning(f"Dead connection detected during send. Cleaning up...")
            await self.cleanup_client(websocket)
            return False

    async def handle_client(self, websocket, path=None):
        remote = getattr(websocket, "remote_address", None)
        client_id = f"{remote[0]}:{remote[1]}" if remote and len(remote) >= 2 else "unknown"
        logger.info(f"Client connected: {client_id}")
        
        try:
            async for message in websocket:
                await self.process_message(websocket, message, client_id)
        except websockets.exceptions.ConnectionClosed:
            pass  # Normal disconnect
        except Exception as e:
            logger.error(f"Error handling client {client_id}: {e}")
        finally:
            await self.cleanup_client(websocket)
            logger.info(f"Client disconnected: {client_id}")

    async def process_message(self, websocket, message: str, client_id: str):
        try:
            msg = json.loads(message)
        except json.JSONDecodeError:
            return
        
        msg_type = msg.get('type')
        
        if msg_type == 'join':
            room_id = msg.get('room')
            role = msg.get('role')
            if not room_id or not role:
                return
            await self.handle_join(websocket, room_id, role, client_id)
            return

        if websocket not in self.connections:
            return
            
        room_id, role = self.connections[websocket]
        
        if msg_type == 'offer':
            await self.relay_offer(websocket, room_id, msg)
        elif msg_type == 'answer':
            await self.relay_answer(websocket, room_id, msg)
        elif msg_type == 'ice':
            await self.relay_ice(websocket, room_id, msg)
        elif msg_type == 'bye':
            await self.cleanup_client(websocket)

    async def handle_join(self, websocket, room_id: str, role: str, client_id: str):
        if room_id not in self.rooms:
            self.rooms[room_id] = SignalingRoom(room_id)
            
        room = self.rooms[room_id]
        
        # FORCE KICK logic: Delete ghosts safely without destroying the room
        if role == 'sender':
            if room.sender and room.sender != websocket:
                if room.sender in self.connections:
                    del self.connections[room.sender]
            room.sender = websocket
        elif role == 'receiver':
            if room.receiver and room.receiver != websocket:
                if room.receiver in self.connections:
                    del self.connections[room.receiver]
            room.receiver = websocket
            
        self.connections[websocket] = (room_id, role)
        logger.info(f"[{room_id}] {role.upper()} joined (client: {client_id})")
        
        await self.safe_send(websocket, json.dumps({"type": "joined", "room": room_id, "role": role}))
        
        # If both are here, trigger the WebRTC pipelines
        if room.is_full():
            logger.info(f"[{room_id}] Room is full (sender + receiver ready). Starting Handshake!")
            await self.safe_send(room.sender, json.dumps({"type": "peer-ready"}))
            await self.safe_send(room.receiver, json.dumps({"type": "peer-ready"}))

    async def relay_offer(self, websocket, room_id: str, msg: dict):
        room = self.rooms.get(room_id)
        if room and room.receiver:
            logger.info(f"[{room_id}] Relaying offer to receiver")
            await self.safe_send(room.receiver, json.dumps({"type": "offer", "sdp": msg.get('sdp')}))

    async def relay_answer(self, websocket, room_id: str, msg: dict):
        room = self.rooms.get(room_id)
        if room and room.sender:
            logger.info(f"[{room_id}] Relaying answer to sender")
            await self.safe_send(room.sender, json.dumps({"type": "answer", "sdp": msg.get('sdp')}))

    async def relay_ice(self, websocket, room_id: str, msg: dict):
        room = self.rooms.get(room_id)
        if not room: return
        
        target = room.receiver if websocket == room.sender else room.sender
        if target:
            await self.safe_send(target, json.dumps({
                "type": "ice",
                "sdpMLineIndex": msg.get('sdpMLineIndex'),
                "candidate": msg.get('candidate')
            }))

    async def cleanup_client(self, websocket):
        if websocket not in self.connections:
            return
        
        room_id, role = self.connections[websocket]
        del self.connections[websocket]
        
        room = self.rooms.get(room_id)
        if room:
            if role == 'sender' and room.sender == websocket:
                room.sender = None
            elif role == 'receiver' and room.receiver == websocket:
                room.receiver = None
            
            logger.info(f"[{room_id}] {role} disconnected")
            
            other = room.receiver if role == 'sender' else room.sender
            if other:
                await self.safe_send(other, json.dumps({"type": "peer-left"}))
            
            if room.is_empty():
                del self.rooms[room_id]
                logger.info(f"[{room_id}] Room destroyed")

    async def start(self):
        logger.info(f"Starting signaling server on ws://{self.host}:{self.port}")
        async with websockets.serve(self.handle_client, self.host, self.port):
            logger.info(f"Signaling server ready")
            await asyncio.Future()

def main():
    import argparse
    parser = argparse.ArgumentParser(description="WebRTC signaling server")
    parser.add_argument("--host", default="0.0.0.0", help="Listen host")
    parser.add_argument("--port", type=int, default=8765, help="Listen port")
    args = parser.parse_args()
    
    server = SignalingServer(args.host, args.port)
    asyncio.run(server.start())

if __name__ == "__main__":
    main()