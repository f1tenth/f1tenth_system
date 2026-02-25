#!/usr/bin/env python3
"""
Minimal WebRTC signaling server using WebSockets.
Routes SDP offers/answers and ICE candidates between sender and receiver peers.
"""

import asyncio
import json
import logging
import sys
from typing import Dict, Set

try:
    import websockets
except ImportError:
    print("Error: websockets library not found. Install with: pip install websockets")
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)


class SignalingRoom:
    """Manages a room with sender and receiver peers."""
    
    def __init__(self, room_id: str):
        self.room_id = room_id
        self.sender = None
        self.receiver = None
    
    def is_full(self) -> bool:
        return self.sender is not None and self.receiver is not None
    
    def is_empty(self) -> bool:
        return self.sender is None and self.receiver is None


class SignalingServer:
    """WebRTC signaling server."""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 8765):
        self.host = host
        self.port = port
        self.rooms: Dict[str, SignalingRoom] = {}
        self.connections: Dict = {}  # ws -> (room_id, role)
    
    async def handle_client(self, websocket, path=None):
        """Handle incoming WebSocket connection.

        Compatible with both websockets handler signatures:
        - old: handle_client(websocket, path)
        - new: handle_client(websocket)
        """
        remote = getattr(websocket, "remote_address", None)
        if remote and len(remote) >= 2:
            client_id = f"{remote[0]}:{remote[1]}"
        else:
            client_id = "unknown"
        logger.info(f"Client connected: {client_id}")
        
        try:
            async for message in websocket:
                await self.process_message(websocket, message, client_id)
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client disconnected: {client_id}")
            await self.cleanup_client(websocket)
        except Exception as e:
            logger.error(f"Error handling client {client_id}: {e}")
            await self.cleanup_client(websocket)
    
    async def process_message(self, websocket, message: str, client_id: str):
        """Process incoming signaling message."""
        try:
            msg = json.loads(message)
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON from {client_id}: {message}")
            return
        
        msg_type = msg.get('type')
        room_id = msg.get('room')
        role = msg.get('role')  # 'sender' or 'receiver'
        
        if not room_id or not role:
            logger.warning(f"Missing room or role in message from {client_id}")
            return
        
        # Join room
        if msg_type == 'join':
            await self.handle_join(websocket, room_id, role, client_id)
        
        # Relay offer
        elif msg_type == 'offer':
            await self.relay_offer(websocket, room_id, msg, client_id)
        
        # Relay answer
        elif msg_type == 'answer':
            await self.relay_answer(websocket, room_id, msg, client_id)
        
        # Relay ICE candidate
        elif msg_type == 'ice':
            await self.relay_ice(websocket, room_id, msg, client_id)
        
        # Disconnect
        elif msg_type == 'bye':
            await self.handle_bye(websocket, room_id, client_id)
        
        else:
            logger.warning(f"Unknown message type from {client_id}: {msg_type}")
    
    async def handle_join(self, websocket, room_id: str, role: str, client_id: str):
        """Handle peer join."""
        if room_id not in self.rooms:
            self.rooms[room_id] = SignalingRoom(room_id)
        
        room = self.rooms[room_id]
        
        # Check role validity
        if role == 'sender':
            if room.sender is not None:
                logger.warning(f"Sender already in room {room_id}, rejecting {client_id}")
                await websocket.send(json.dumps({"type": "error", "message": "Sender already in room"}))
                return
            room.sender = websocket
        elif role == 'receiver':
            if room.receiver is not None:
                logger.warning(f"Receiver already in room {room_id}, rejecting {client_id}")
                await websocket.send(json.dumps({"type": "error", "message": "Receiver already in room"}))
                return
            room.receiver = websocket
        else:
            logger.warning(f"Invalid role from {client_id}: {role}")
            return
        
        self.connections[websocket] = (room_id, role)
        logger.info(f"[{room_id}] {role.upper()} joined (client: {client_id})")
        
        # Send acknowledgment
        await websocket.send(json.dumps({"type": "joined", "room": room_id, "role": role}))
        
        # If room is full, notify both peers
        if room.is_full():
            logger.info(f"[{room_id}] Room is full (sender + receiver ready)")
            await room.sender.send(json.dumps({"type": "peer-ready"}))
            await room.receiver.send(json.dumps({"type": "peer-ready"}))
    
    async def relay_offer(self, websocket, room_id: str, msg: dict, client_id: str):
        """Relay SDP offer from sender to receiver."""
        if room_id not in self.rooms:
            logger.warning(f"Room {room_id} not found")
            return
        
        room = self.rooms[room_id]
        if room.receiver is None:
            logger.warning(f"[{room_id}] No receiver to send offer to")
            return
        
        sdp = msg.get('sdp')
        if not sdp:
            logger.warning(f"[{room_id}] Offer missing SDP")
            return
        
        logger.info(f"[{room_id}] Relaying offer from sender")
        await room.receiver.send(json.dumps({"type": "offer", "sdp": sdp}))
    
    async def relay_answer(self, websocket, room_id: str, msg: dict, client_id: str):
        """Relay SDP answer from receiver to sender."""
        if room_id not in self.rooms:
            logger.warning(f"Room {room_id} not found")
            return
        
        room = self.rooms[room_id]
        if room.sender is None:
            logger.warning(f"[{room_id}] No sender to send answer to")
            return
        
        sdp = msg.get('sdp')
        if not sdp:
            logger.warning(f"[{room_id}] Answer missing SDP")
            return
        
        logger.info(f"[{room_id}] Relaying answer from receiver")
        await room.sender.send(json.dumps({"type": "answer", "sdp": sdp}))
    
    async def relay_ice(self, websocket, room_id: str, msg: dict, client_id: str):
        """Relay ICE candidate."""
        if room_id not in self.rooms:
            logger.warning(f"Room {room_id} not found")
            return
        
        room = self.rooms[room_id]
        candidate = msg.get('candidate')
        sdp_mline_index = msg.get('sdpMLineIndex')
        
        if candidate is None or sdp_mline_index is None:
            logger.warning(f"[{room_id}] ICE candidate missing fields")
            return
        
        # Determine sender/receiver and relay to the other
        if websocket == room.sender:
            target = room.receiver
            source = "sender"
        elif websocket == room.receiver:
            target = room.sender
            source = "receiver"
        else:
            logger.warning(f"[{room_id}] Unknown source for ICE candidate")
            return
        
        if target is None:
            logger.debug(f"[{room_id}] No target peer for ICE candidate from {source}")
            return
        
        logger.debug(f"[{room_id}] Relaying ICE from {source}")
        await target.send(json.dumps({
            "type": "ice",
            "sdpMLineIndex": sdp_mline_index,
            "candidate": candidate
        }))
    
    async def handle_bye(self, websocket, room_id: str, client_id: str):
        """Handle peer disconnect."""
        logger.info(f"[{room_id}] Peer disconnect from {client_id}")
        await self.cleanup_client(websocket)
    
    async def cleanup_client(self, websocket):
        """Clean up client connection."""
        if websocket not in self.connections:
            return
        
        room_id, role = self.connections[websocket]
        del self.connections[websocket]
        
        if room_id not in self.rooms:
            return
        
        room = self.rooms[room_id]
        
        # Remove peer from room
        if role == 'sender':
            room.sender = None
        elif role == 'receiver':
            room.receiver = None
        
        logger.info(f"[{room_id}] {role} disconnected")
        
        # Notify other peer
        if role == 'sender' and room.receiver:
            try:
                await room.receiver.send(json.dumps({"type": "peer-left"}))
            except:
                pass
        elif role == 'receiver' and room.sender:
            try:
                await room.sender.send(json.dumps({"type": "peer-left"}))
            except:
                pass
        
        # Clean up empty room
        if room.is_empty():
            del self.rooms[room_id]
            logger.info(f"[{room_id}] Room destroyed")
    
    async def start(self):
        """Start the signaling server."""
        logger.info(f"Starting signaling server on ws://{self.host}:{self.port}")
        async with websockets.serve(self.handle_client, self.host, self.port):
            logger.info(f"Signaling server ready")
            await asyncio.Future()  # run forever


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
