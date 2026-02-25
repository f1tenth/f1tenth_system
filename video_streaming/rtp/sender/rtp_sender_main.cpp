#include "gstreamer_sender_rtp.hpp"

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
            << " [--host <ip>] [--port <port>] [--source <videotestsrc|v4l2>]"
            << " [--device <path>] [--width <px>] [--height <px>] [--fps <n>]"
            << std::endl;
}
}  // namespace

int main(int argc, char* argv[]) {
  std::string host = "127.0.0.1";
  int port = 5000;
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

    if (arg == "--host") {
      char* value = require_value("--host");
      if (!value) {
        return 1;
      }
      host = value;
    } else if (arg == "--port") {
      char* value = require_value("--port");
      if (!value) {
        return 1;
      }
      port = std::stoi(value);
    } else if (arg == "--source") {
      char* value = require_value("--source");
      if (!value) {
        return 1;
      }
      source = value;
    } else if (arg == "--device") {
      char* value = require_value("--device");
      if (!value) {
        return 1;
      }
      device = value;
    } else if (arg == "--width") {
      char* value = require_value("--width");
      if (!value) {
        return 1;
      }
      width = std::stoi(value);
    } else if (arg == "--height") {
      char* value = require_value("--height");
      if (!value) {
        return 1;
      }
      height = std::stoi(value);
    } else if (arg == "--fps") {
      char* value = require_value("--fps");
      if (!value) {
        return 1;
      }
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

  if (port <= 0 || port > 65535 || width <= 0 || height <= 0 || fps <= 0) {
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
  track_info.destination_host = host;
  track_info.destination_port = port;

  std::signal(SIGINT, on_signal);
#ifdef SIGTERM
  std::signal(SIGTERM, on_signal);
#endif

  std::cout << "Starting RTP sender -> " << host << ":" << port << std::endl;
  std::cout << "Source: " << source << ", Resolution: " << width << "x" << height
            << " @ " << fps << " FPS" << std::endl;

  VideoOutputPipeline pipeline(track_info);
  pipeline.start();

  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  pipeline.stop();
  std::cout << "RTP sender stopped." << std::endl;
  return 0;
}
