# Video Streaming Device Run Guide

This guide assumes each role runs on a separate device.

## 1) RTP Sender Device (camera/transmitter)

Build image:

```bash
docker build -t f1-rtp-sender ./video_streaming/rtp/sender
```

Run image (replace `RECEIVER_IP`):

```bash
docker run --rm \
  --device /dev/video0:/dev/video0 \
  f1-rtp-sender \
  --source v4l2 \
  --device /dev/video0 \
  --host RECEIVER_IP \
  --port 5000 \
  --width 1280 \
  --height 720 \
  --fps 30
```

For a no-camera test pattern, use:

```bash
docker run --rm f1-rtp-sender --source videotestsrc --host RECEIVER_IP --port 5000
```

## 2) RTP Receiver Device (display)

Build image:

```bash
docker build -t f1-rtp-receiver ./video_streaming/rtp/receiver
```

Run image (port must match sender):

```bash
docker run --rm -it \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  f1-rtp-receiver \
  --port 5000 \
  --payload-type 102
```

## 3) WebRTC Signaling Server Device

Build image (uses WebRTC receiver folder image, command overridden to signaling server):

```bash
docker build -t f1-webrtc-receiver ./video_streaming/webrtc/receiver
```

Run signaling server:

```bash
docker run --rm -it \
  -p 8765:8765 \
  --entrypoint python3 \
  f1-webrtc-receiver \
  /app/signaling_server.py --host 0.0.0.0 --port 8765
```

## 4) WebRTC Sender Device

Build image:

```bash
docker build -t f1-webrtc-sender ./video_streaming/webrtc/sender
```

Run image (replace `SIGNALING_IP`):

```bash
docker run --rm \
  --device /dev/video0:/dev/video0 \
  f1-webrtc-sender \
  --server-url ws://SIGNALING_IP:8765 \
  --room-id f1tenth \
  --source v4l2 \
  --device /dev/video0
```

## 5) WebRTC Receiver Device

Current `webrtc/receiver/gstreamer_receiver_webrtc.py` script is still an RTP-style receiver pipeline. If you run it, it listens on UDP RTP (`--port`, payload 102), not full `webrtcbin` signaling yet.

Build + run as currently implemented:

```bash
docker build -t f1-webrtc-receiver ./video_streaming/webrtc/receiver

docker run --rm -it \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  f1-webrtc-receiver \
  --port 5000
```

## Network checklist

- Sender can reach receiver/signaling IPs (`ping`, firewall open)
- RTP UDP port open (default `5000`)
- Signaling TCP port open (default `8765`)
- Matching room id for WebRTC sender/receiver peers (default `f1tenth`)
