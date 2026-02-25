#!/usr/bin/env python3
"""
WebRTC signaling client for receiver (Python).
Handles WebSocket connection to signaling server and routes SDP/ICE messages.
"""

import asyncio
import json
import logging
from typing import Callable, Optional

try:
    import websockets
except ImportError:
    print("Error: websockets library not found. Install with: pip install websockets")
    raise

logger = logging.getLogger(__name__)


class SignalingClient:
    """WebRTC signaling client for receiver."""
    
    def __init__(self, room_id: str):
        """
        Initialize signaling client.
        
        Args:
            room_id: Room identifier (shared between sender and receiver)
        """
        self.room_id = room_id
        self.role = "receiver"
        self.ws = None
        self.connected = False
        
        # Callbacks
        self.on_peer_ready: Optional[Callable[[], None]] = None
        self.on_remote_offer: Optional[Callable[[str], None]] = None  # sdp
        self.on_remote_ice: Optional[Callable[[int, str], None]] = None  # sdp_mline_index, candidate
        self.on_error: Optional[Callable[[str], None]] = None
        self.on_connected: Optional[Callable[[], None]] = None
        self.on_disconnected: Optional[Callable[[], None]] = None
    
    async def connect(self, server_url: str) -> bool:
        """
        Connect to signaling server.
        
        Args:
            server_url: WebSocket URL (e.g., "ws://192.168.1.100:8765")
        
        Returns:
            True if connection initiated
        """
        if self.connected:
            logger.warning("Already connected")
            return False
        
        try:
            logger.info(f"Connecting to {server_url} (room: {self.room_id})")
            self.ws = await websockets.connect(server_url)
            self.connected = True
            
            # Send join message
            join_msg = {
                "type": "join",
                "room": self.room_id,
                "role": self.role
            }
            await self.ws.send(json.dumps(join_msg))
            
            if self.on_connected:
                self.on_connected()
            
            # Start message loop
            asyncio.create_task(self._message_loop())
            
            return True
        
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            if self.on_error:
                self.on_error(f"Connection failed: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from server."""
        if not self.connected:
            return
        
        try:
            bye_msg = {
                "type": "bye",
                "room": self.room_id,
                "role": self.role
            }
            await self.ws.send(json.dumps(bye_msg))
        except Exception as e:
            logger.debug(f"Error sending bye: {e}")
        
        try:
            await self.ws.close()
        except:
            pass
        
        self.connected = False
        if self.on_disconnected:
            self.on_disconnected()
    
    async def send_answer(self, sdp: str):
        """
        Send local SDP answer to peer.
        
        Args:
            sdp: SDP content (text format)
        """
        if not self.connected:
            logger.warning("Not connected, cannot send answer")
            return
        
        try:
            msg = {
                "type": "answer",
                "room": self.room_id,
                "role": self.role,
                "sdp": sdp
            }
            await self.ws.send(json.dumps(msg))
        except Exception as e:
            logger.error(f"Failed to send answer: {e}")
            if self.on_error:
                self.on_error(f"Failed to send answer: {e}")
    
    async def send_ice_candidate(self, sdp_mline_index: int, candidate: str):
        """
        Send local ICE candidate to peer.
        
        Args:
            sdp_mline_index: Media line index (usually 0)
            candidate: ICE candidate string
        """
        if not self.connected:
            return  # Silent fail for ICE candidates
        
        try:
            msg = {
                "type": "ice",
                "room": self.room_id,
                "role": self.role,
                "sdpMLineIndex": sdp_mline_index,
                "candidate": candidate
            }
            await self.ws.send(json.dumps(msg))
        except Exception as e:
            logger.debug(f"Failed to send ICE candidate: {e}")
    
    async def _message_loop(self):
        """Internal: Listen for messages from server."""
        try:
            async for message in self.ws:
                try:
                    msg = json.loads(message)
                    await self._handle_message(msg)
                except json.JSONDecodeError:
                    logger.warning(f"Invalid JSON: {message}")
                except Exception as e:
                    logger.error(f"Error processing message: {e}")
        
        except websockets.exceptions.ConnectionClosed:
            logger.info("Connection closed by server")
        except Exception as e:
            logger.error(f"Message loop error: {e}")
        
        finally:
            self.connected = False
            if self.on_disconnected:
                self.on_disconnected()
    
    async def _handle_message(self, msg: dict):
        """Internal: Handle incoming signaling message."""
        msg_type = msg.get("type")
        
        if msg_type == "joined":
            logger.info(f"Joined room {self.room_id} as {self.role}")
        
        elif msg_type == "peer-ready":
            logger.info("Peer (sender) is ready")
            if self.on_peer_ready:
                self.on_peer_ready()
        
        elif msg_type == "offer":
            sdp = msg.get("sdp", "")
            logger.info("Received offer from peer")
            if self.on_remote_offer:
                self.on_remote_offer(sdp)
        
        elif msg_type == "ice":
            sdp_mline_index = msg.get("sdpMLineIndex", 0)
            candidate = msg.get("candidate", "")
            logger.debug(f"Received ICE candidate (mline {sdp_mline_index})")
            if self.on_remote_ice:
                self.on_remote_ice(sdp_mline_index, candidate)
        
        elif msg_type == "error":
            error_msg = msg.get("message", "Unknown error")
            logger.error(f"Server error: {error_msg}")
            if self.on_error:
                self.on_error(error_msg)
        
        elif msg_type == "peer-left":
            logger.warning("Peer disconnected")
            if self.on_error:
                self.on_error("Peer disconnected")
        
        else:
            logger.debug(f"Unknown message type: {msg_type}")


# ============================================================================
# Integration Example with GStreamer Pipeline
# ============================================================================

class ReceiverPipeline:
    """Example: GStreamer receiver with WebRTC signaling integration."""
    
    def __init__(self, room_id: str):
        """Initialize receiver pipeline (WebRTC + GStreamer)."""
        self.room_id = room_id
        self.signaling = SignalingClient(room_id)
        self.webrtcbin = None  # Will be GStreamer webrtcbin element
        self.pipeline = None   # Will be GStreamer pipeline
        
        # Setup signaling callbacks
        self.signaling.on_peer_ready = self._on_peer_ready
        self.signaling.on_remote_offer = self._on_remote_offer
        self.signaling.on_remote_ice = self._on_remote_ice
        self.signaling.on_error = self._on_error
    
    async def connect_signaling(self, server_url: str) -> bool:
        """Connect to signaling server."""
        return await self.signaling.connect(server_url)
    
    def _on_peer_ready(self):
        """Peer is ready; waiting for offer."""
        logger.info("Peer ready - waiting for SDP offer")
    
    def _on_remote_offer(self, sdp: str):
        """Received SDP offer from sender."""
        logger.info("Received offer from sender")
        # TODO: Parse SDP, set remote description in webrtcbin
        # then create answer and call self.signaling.send_answer(answer_sdp)
    
    def _on_remote_ice(self, sdp_mline_index: int, candidate: str):
        """Received ICE candidate from sender."""
        logger.debug(f"Received ICE candidate (mline {sdp_mline_index})")
        # TODO: Add to webrtcbin via "add-ice-candidate" signal
    
    def _on_error(self, error_msg: str):
        """Error occurred."""
        logger.error(f"Signaling error: {error_msg}")
    
    async def run(self, server_url: str):
        """Run receiver (signaling + GStreamer events)."""
        if not await self.connect_signaling(server_url):
            logger.error("Failed to connect signaling")
            return
        
        # TODO: Create GStreamer pipeline here
        # Start main event loop (GStreamer + async)
        
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            logger.info("Shutting down")
            await self.signaling.disconnect()


# ============================================================================
# Minimal Test Client
# ============================================================================

async def test_client():
    """Simple test of signaling client (without GStreamer)."""
    client = SignalingClient("test-room")
    
    # Setup callbacks
    client.on_connected = lambda: print("[CALLBACK] Connected")
    client.on_peer_ready = lambda: print("[CALLBACK] Peer ready")
    client.on_remote_offer = lambda sdp: print(f"[CALLBACK] Offer received ({len(sdp)} bytes)")
    client.on_remote_ice = lambda mline, cand: print(f"[CALLBACK] ICE candidate (mline {mline})")
    client.on_error = lambda msg: print(f"[CALLBACK] Error: {msg}")
    client.on_disconnected = lambda: print("[CALLBACK] Disconnected")
    
    # Connect to server
    success = await client.connect("ws://localhost:8765")
    if not success:
        print("Failed to connect")
        return
    
    # Keep running
    try:
        await asyncio.sleep(60)
    except KeyboardInterrupt:
        pass
    finally:
        await client.disconnect()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(test_client())
