import argparse
import asyncio
import logging
import os
import signal
import time

import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstSdp, GstWebRTC

from signaling_client_python import SignalingClient


logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)


FRAME_LOG_INTERVAL = 150


class WebRTCReceiver:
    """WebRTC H.264 receiver using webrtcbin + signaling server."""

    def __init__(self, server_url: str, room_id: str, use_hw_decode: bool = True):
        Gst.init(None)

        self.server_url = server_url
        self.room_id = room_id
        self.use_hw_decode = use_hw_decode
        self.running = True

        self.loop = asyncio.get_running_loop()
        self.signaling = SignalingClient(room_id)
        self.signaling.on_connected = self._on_signaling_connected
        self.signaling.on_disconnected = self._on_signaling_disconnected
        self.signaling.on_remote_offer = self._on_remote_offer
        self.signaling.on_remote_ice = self._on_remote_ice
        self.signaling.on_error = self._on_signaling_error

        self.frame_count = 0
        self.last_frame_time = None

        self.pipeline = Gst.Pipeline.new("webrtc-receiver")
        self.webrtcbin = Gst.ElementFactory.make("webrtcbin", "webrtc")
        if not self.webrtcbin:
            raise RuntimeError("Failed to create webrtcbin")

        self.webrtcbin.set_property("stun-server", "stun://stun.l.google.com:19302")
        self.webrtcbin.connect("on-ice-candidate", self._on_local_ice_candidate)
        self.webrtcbin.connect("pad-added", self._on_incoming_stream)

        # Force specific IP for Docker-on-Windows workarounds
        # When running in Docker, we need to manually specify the HOST IP
        # so ICE candidates are valid for the remote peer.
        external_ip = os.getenv("RTP_EXTERNAL_IP")
        if external_ip:
            logger.info("Forcing ICE Candidate IP: %s (RTP_EXTERNAL_IP set)", external_ip)
            # Add a local candidate manually for the host IP
            # Wait for webrtcbin to be ready before adding candidate (done in run loop or separate task)
            self.forced_ip = external_ip
        else:
            self.forced_ip = None

        self.pipeline.add(self.webrtcbin)
        self.pipeline.set_state(Gst.State.PLAYING)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        logger.info("WebRTC receiver initialized")

    async def start(self):
        ok = await self.signaling.connect(self.server_url)
        if not ok:
            raise RuntimeError("Failed to connect to signaling server")

    async def run(self):
        logger.info("Receiver running. Waiting for offer...")
        
        # If we have a forced external IP, add it as a candidate manually
        if self.forced_ip:
             # Basic host candidate format for video (UDP and TCP)
             # component=1 (RTP)
             # priority: arbitrary high
             asyncio.create_task(self._announce_forced_ip())

        try:
            while self.running:
                await asyncio.sleep(0.02)
        finally:
            await self.shutdown()

    async def _announce_forced_ip(self):
        # Announce the forced IP as a host candidate to the signaling server
        # This tricks the remote peer into trying to connect to the host IP
        # instead of the internal Docker IP
        await asyncio.sleep(1) # Wait for connection
        
        # Add basic candidates for RTP (UDP) and control (if needed)
        # We generate a fake candidate string and send it via signaling
        # We don't add it to webrtcbin because we assume port forwarding handles the traffic
        # But we DO need to know which port webrtcbin picked... 
        # Since webrtcbin picks random ports, we should really force the port range in docker-compose
        pass

    async def shutdown(self):
        if not self.running:
            return
        self.running = False
        logger.info("Shutting down receiver")
        await self.signaling.disconnect()
        self.pipeline.set_state(Gst.State.NULL)

    def _on_bus_message(self, _, message):
        msg_type = message.type
        if msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error("GStreamer error: %s (%s)", err, debug)
            self.running = False
        elif msg_type == Gst.MessageType.EOS:
            logger.info("GStreamer EOS")
            self.running = False

    def _on_signaling_connected(self):
        logger.info("Connected to signaling server (%s), room=%s", self.server_url, self.room_id)
        logger.info("DISPLAY=%s GST_VIDEO_SINK=%s", os.getenv("DISPLAY", "(unset)"), os.getenv("GST_VIDEO_SINK", "autovideosink"))

    def _on_signaling_disconnected(self):
        logger.info("Disconnected from signaling server")

    def _on_signaling_error(self, error: str):
        logger.error("Signaling error: %s", error)

    def _on_local_ice_candidate(self, _webrtcbin, mlineindex, candidate):
        if not candidate:
            return

        # If we have a forced external IP (e.g. from Docker host),
        # parse the candidate string and replace the internal IP
        # with the forced external IP before sending it.
        # Format: candidate:foundation component protocol priority ip port type ...
        if self.forced_ip:
            parts = candidate.split()
            if len(parts) >= 8 and parts[7] == "host":
                # Replace all host candidates with the forced external IP
                # Keep the port, assume it's mapped 1:1 via docker
                old_ip = parts[4]
                parts[4] = self.forced_ip
                modified_candidate = " ".join(parts)
                logger.info("Modifying ICE candidate: %s -> %s", old_ip, self.forced_ip)
                candidate = modified_candidate
            
        asyncio.run_coroutine_threadsafe(
            self.signaling.send_ice_candidate(int(mlineindex), candidate),
            self.loop,
        )

    def _on_remote_offer(self, sdp_text: str):
        logger.info("Received remote SDP offer (%d bytes)", len(sdp_text))

        video_caps = Gst.Caps.from_string(
            "application/x-rtp,media=video,encoding-name=H264,payload=102,clock-rate=90000"
        )
        self.webrtcbin.emit(
            "add-transceiver",
            GstWebRTC.WebRTCRTPTransceiverDirection.RECVONLY,
            video_caps,
        )

        res, sdp = GstSdp.SDPMessage.new()
        if res != GstSdp.SDPResult.OK:
            logger.error("Failed to create SDP message")
            return

        parse_res = GstSdp.sdp_message_parse_buffer(bytes(sdp_text.encode("utf-8")), sdp)
        if parse_res != GstSdp.SDPResult.OK:
            logger.error("Failed to parse remote SDP offer")
            return

        offer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.OFFER, sdp)
        set_remote_promise = Gst.Promise.new()
        self.webrtcbin.emit("set-remote-description", offer, set_remote_promise)
        set_remote_promise.interrupt()

        answer_promise = Gst.Promise.new()
        self.webrtcbin.emit("create-answer", None, answer_promise)
        answer_promise.wait()
        reply = answer_promise.get_reply()
        answer = reply.get_value("answer")

        set_local_promise = Gst.Promise.new()
        self.webrtcbin.emit("set-local-description", answer, set_local_promise)
        set_local_promise.interrupt()

        answer_sdp_text = answer.sdp.as_text()
        asyncio.run_coroutine_threadsafe(
            self.signaling.send_answer(answer_sdp_text),
            self.loop,
        )

        logger.info("Created and sent SDP answer (%d bytes)", len(answer_sdp_text))

    def _on_remote_ice(self, sdp_mline_index: int, candidate: str):
        self.webrtcbin.emit("add-ice-candidate", int(sdp_mline_index), candidate)

    def _on_incoming_stream(self, _webrtcbin, pad):
        caps = pad.get_current_caps()
        if not caps:
            return
        name = caps.get_structure(0).get_name()
        if name != "application/x-rtp":
            return

        logger.info("Incoming RTP stream pad detected: %s", name)

        queue = Gst.ElementFactory.make("queue", None)
        depay = Gst.ElementFactory.make("rtph264depay", None)
        parse = Gst.ElementFactory.make("h264parse", None)

        decoder_name = "nvh264dec" if self.use_hw_decode else "avdec_h264"
        decoder = Gst.ElementFactory.make(decoder_name, None)
        if not decoder:
            decoder = Gst.ElementFactory.make("avdec_h264", None)
            logger.warning("Falling back to avdec_h264")

        convert = Gst.ElementFactory.make("videoconvert", None)
        sink_name = os.getenv("GST_VIDEO_SINK", "autovideosink")
        sink = Gst.ElementFactory.make(sink_name, None)
        if not sink:
            logger.warning("Failed to create sink '%s', falling back to autovideosink", sink_name)
            sink = Gst.ElementFactory.make("autovideosink", None)

        if not all([queue, depay, parse, decoder, convert, sink]):
            logger.error("Failed to create decode/render elements")
            return

        sink.set_property("sync", False)
        logger.info("Using video sink: %s", sink.get_factory().get_name() if sink.get_factory() else "unknown")

        self.pipeline.add(queue)
        self.pipeline.add(depay)
        self.pipeline.add(parse)
        self.pipeline.add(decoder)
        self.pipeline.add(convert)
        self.pipeline.add(sink)

        queue.sync_state_with_parent()
        depay.sync_state_with_parent()
        parse.sync_state_with_parent()
        decoder.sync_state_with_parent()
        convert.sync_state_with_parent()
        sink.sync_state_with_parent()

        if not Gst.Element.link_many(queue, depay, parse, decoder, convert, sink):
            logger.error("Failed to link incoming decode chain")
            return

        queue_sink_pad = queue.get_static_pad("sink")
        if pad.link(queue_sink_pad) != Gst.PadLinkReturn.OK:
            logger.error("Failed to link webrtc pad to decode queue")
            return

        sink_pad = sink.get_static_pad("sink")
        sink_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_frame_probe)
        logger.info("Incoming stream linked to decoder and sink")

    def _on_frame_probe(self, _pad, _info):
        self.frame_count += 1
        self.last_frame_time = time.time()
        if self.frame_count % FRAME_LOG_INTERVAL == 0:
            logger.info("Received %d frames", self.frame_count)
        return Gst.PadProbeReturn.OK


async def async_main():
    parser = argparse.ArgumentParser(description="WebRTC H.264 receiver")
    parser.add_argument("--server-url", default="ws://localhost:8765", help="Signaling server URL")
    parser.add_argument("--room-id", default="f1tenth", help="Signaling room ID")
    parser.add_argument("--hw", action="store_true", default=True, help="Use hardware H.264 decoder when available")
    parser.add_argument("--sw", action="store_true", help="Force software decoder")
    args = parser.parse_args()

    use_hw = args.hw and not args.sw
    receiver = WebRTCReceiver(args.server_url, args.room_id, use_hw_decode=use_hw)

    stop_event = asyncio.Event()

    def stop_handler(*_):
        stop_event.set()

    signal.signal(signal.SIGINT, stop_handler)
    try:
        signal.signal(signal.SIGTERM, stop_handler)
    except Exception:
        pass

    await receiver.start()

    runner = asyncio.create_task(receiver.run())
    stopper = asyncio.create_task(stop_event.wait())

    done, pending = await asyncio.wait({runner, stopper}, return_when=asyncio.FIRST_COMPLETED)
    for task in pending:
        task.cancel()

    if stop_event.is_set():
        await receiver.shutdown()

    if runner in done:
        runner.result()


if __name__ == "__main__":
    asyncio.run(async_main())