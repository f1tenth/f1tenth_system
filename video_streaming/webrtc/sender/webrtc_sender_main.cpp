#include "gstreamer_sender_webrtc.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::atomic_bool g_running{true};

void on_signal(int) {
  g_running = false;
}

void print_usage(const char* program_name) {
  std::cout << "Usage: " << program_name
            << " [--server-url <ws://host:port>] [--room-id <id>]"
            << " [--host <ip>] [--port <port>]"
            << " [--source <videotestsrc|v4l2>] [--device <path>]"
            << " [--width <px>] [--height <px>] [--fps <n>]"
            << std::endl;
}
}

int main(int argc, char* argv[]) {
  std::string server_url = "ws://host.docker.internal:8765";
  std::string room_id = "f1tenth";
  std::string source = "videotestsrc";
  std::string device = "/dev/video0";
  int width = 1280;
  int height = 720;
  int fps = 30;

  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];

    auto require_value = [&](const char* option) -> char* {
      if (index + 1 >= argc) {
        std::cerr << "Missing value for " << option << std::endl;
        print_usage(argv[0]);
        return nullptr;
      }
      return argv[++index];
    };

    if (arg == "--server-url") {
      char* value = require_value("--server-url");
      if (!value) return 1;
      server_url = value;
    } else if (arg == "--room-id") {
      char* value = require_value("--room-id");
      if (!value) return 1;
      room_id = value;
    } else if (arg == "--source") {
      char* value = require_value("--source");
      if (!value) return 1;
      source = value;
    } else if (arg == "--device") {
      char* value = require_value("--device");
      if (!value) return 1;
      device = value;
    } else if (arg == "--width") {
      char* value = require_value("--width");
      if (!value) return 1;
      width = std::stoi(value);
    } else if (arg == "--height") {
      char* value = require_value("--height");
      if (!value) return 1;
      height = std::stoi(value);
    } else if (arg == "--fps") {
      char* value = require_value("--fps");
      if (!value) return 1;
      fps = std::stoi(value);
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      print_usage(argv[0]);
      return 1;
    }
  }

  if (width <= 0 || height <= 0 || fps <= 0) {
    std::cerr << "Invalid numeric argument." << std::endl;
    return 1;
  }

  gst_init(&argc, &argv);

  VideoOutputTrackInfo track_info;
  track_info.source = source == "v4l2" ? VideoSource::v4l2 : VideoSource::videotestsrc;
  track_info.device = device;
  track_info.media_type = "video/x-raw";
  track_info.caps.width = width;
  track_info.caps.height = height;
  track_info.caps.framerate.num = fps;
  track_info.caps.framerate.den = 1;
  track_info.codec = VideoCodec::H264;

  std::signal(SIGINT, on_signal);
#ifdef SIGTERM
  std::signal(SIGTERM, on_signal);
#endif

  VideoOutputPipeline pipeline(track_info);
  
  // Connect to signaling BEFORE starting pipeline to ensure we don't clear caps filter too early
  if (!pipeline.connect_signaling(server_url, room_id)) {
    std::cerr << "Failed to connect to signaling server" << std::endl;
    return 1;
  }
  
  std::cout << "Signaling connected, waiting for peer..." << std::endl;
  
  // Start pipeline now
  pipeline.start();

  std::cout << "WebRTC sender started. Signaling: " << server_url
            << ", room: " << room_id << std::endl;

  while (g_running.load()) {
    pipeline.process_signaling_messages();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  pipeline.stop();
  std::cout << "WebRTC sender stopped." << std::endl;
  return 0;
}
