import argparse
import time
import signal
import pyzed.sl as sl
import cv2
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class GStreamerStreamer:
    """GStreamer RTP H.264 streamer with raw video input."""
    
    def __init__(self, host: str, port: int, width: int, height: int, fps: int):
        Gst.init(None)
        
        pipeline_desc = (
            f"appsrc name=src is-live=true do-timestamp=true format=time "
            f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "
            f"nvh264enc bitrate=4000 ! "
            f"h264parse config-interval=1 ! "
            f"rtph264pay pt=96 mtu=1400 config-interval=1 ! "
            f"udpsink host={host} port={port} sync=false async=false"
        )
        
        self.pipeline = Gst.parse_launch(pipeline_desc)
        self.appsrc = self.pipeline.get_by_name('src')
        self.pipeline.set_state(Gst.State.PLAYING)
        self.frame_count = 0
        self.fps = fps
        
        print(f"GStreamer RTP streaming to {host}:{port}")
    
    def send_frame(self, frame_bgr: bytes):
        """Send BGR frame data to GStreamer pipeline."""
        if not frame_bgr or len(frame_bgr) == 0:
            return
        
        # Create GStreamer buffer
        buf = Gst.Buffer.new_allocate(None, len(frame_bgr), None)
        buf.fill(0, frame_bgr)
        buf.pts = self.frame_count * (1000000000 // self.fps)  # nanoseconds
        buf.duration = 1000000000 // self.fps
        
        # Push buffer
        retval = self.appsrc.emit('push-buffer', buf)
        if retval != Gst.FlowReturn.OK:
            print(f"Error pushing buffer: {retval}")
        
        self.frame_count += 1
    
    def stop(self):
        """Stop GStreamer pipeline."""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)


def create_zed_camera(args: argparse.Namespace):
    """Create and configure ZED camera."""
    zed = sl.Camera()
    init_params = sl.InitParameters()
    
    # Map resolution
    resolution_map = {
        720: sl.RESOLUTION.HD720,
        1080: sl.RESOLUTION.HD1080,
        2208: sl.RESOLUTION.HD2K,
    }
    init_params.camera_resolution = resolution_map.get(args.height, sl.RESOLUTION.HD720)
    init_params.camera_fps = args.fps
    init_params.sdk_gpu_id = 0
    
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("Failed to open ZED camera")
    
    print("[ZED] Camera successfully opened")
    print(f"[ZED] Serial Number: {zed.get_camera_information().serial_number}")
    
    return zed


def main():
    parser = argparse.ArgumentParser(description="ZED camera with H.264 RTP streaming")
    parser.add_argument("--width", type=int, default=1280, help="Frame width")
    parser.add_argument("--height", type=int, default=720, help="Frame height")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate")
    parser.add_argument("--stream-host", type=str, default="127.0.0.1", help="RTP destination host")
    parser.add_argument("--stream-port", type=int, default=5000, help="RTP destination port")
    parser.add_argument("--retry-interval", type=int, default=10, help="Camera retry interval (seconds)")
    args = parser.parse_args()
    
    print(f"ZED SDK version: {sl.Camera.get_sdk_version()}")
    print(f"Configuration: {args.width}x{args.height} @ {args.fps}fps")
    
    # Initialize GStreamer streamer
    streamer = GStreamerStreamer(args.stream_host, args.stream_port, args.width, args.height, args.fps)
    print(f"Streaming to {args.stream_host}:{args.stream_port}")
    print("Press Ctrl+C to quit")
    
    zed = None
    image = sl.Mat()
    
    def signal_handler(sig, frame):
        print("\nInterrupted by user")
        if zed:
            zed.close()
        streamer.stop()
        exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        while True:
            retry_count = 0
            
            # Camera initialization/reconnection loop
            while True:
                try:
                    if zed:
                        zed.close()
                    zed = create_zed_camera(args)
                    print("Camera started")
                    break
                except Exception as e:
                    retry_count += 1
                    print(f"Failed to initialize camera (attempt {retry_count}): {e}")
                    print(f"Retrying in {args.retry_interval} seconds...")
                    try:
                        time.sleep(args.retry_interval)
                    except KeyboardInterrupt:
                        print("\nInterrupted by user during retry wait")
                        raise
            
            # Main streaming loop
            try:
                frame_count = 0
                while True:
                    if zed.grab() == sl.ERROR_CODE.SUCCESS:
                        # Retrieve image from ZED
                        zed.retrieve_image(image, sl.VIEW.LEFT)
                        frame = image.get_data()
                        
                        # Convert BGRA to BGR
                        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                        
                        # Send frame
                        streamer.send_frame(frame_bgr.tobytes())
                        
                        frame_count += 1
                        if frame_count % 150 == 0:
                            print(f"Streamed {frame_count} frames")
                    else:
                        print("Failed to grab frame from ZED")
                        break
                
                # If we exit the loop, camera issue occurred
                print("Camera stream stopped unexpectedly. Attempting to reconnect...")
                
            except Exception as e:
                print(f"Streaming error: {e}. Attempting to reconnect...")
                    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if zed:
            zed.close()
        streamer.stop()
        print("Cleanup complete")


if __name__ == "__main__":
    main()