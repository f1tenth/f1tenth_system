#ifndef GSTREAMER_VIDEO_OUTPUT_PIPELINE_HPP
#define GSTREAMER_VIDEO_OUTPUT_PIPELINE_HPP

#define H264_PAYLOAD_TYPE 102
#define VP9_PAYLOAD_TYPE 100
#define VP8_PAYLOAD_TYPE 96
#define X264_ENC_PRESET_ULTRAFAST 1
#define X264_ENC_TUNE_ZEROLATENCY 4
#define RTP_MTU 1200

#include <gst/gst.h>

#include <string>

struct AVRational {
  int num;
  int den;
};

enum class VideoCodec {
  H264 = 0,
  VP8 = 1,
};

enum class VideoSource {
  v4l2 = 0,
  navargus = 1,
  videotestsrc = 2,
};

struct VideoCaps {
  int width;
  int height;
  AVRational framerate;
};

struct VideoOutputTrackInfo {
  VideoSource source;
  std::string device;
  std::string media_type;
  VideoCaps caps;
  VideoCodec codec;
};

/**
 * A GStreamer pipeline that encodes a video stream from a media device.
 */
class VideoOutputPipeline {
 public:
  VideoOutputPipeline(VideoOutputTrackInfo info);

  ~VideoOutputPipeline();

  void start();

  void stop();

  inline GstElement* get_sink() const { return sink_; }

 private:
  VideoOutputTrackInfo info_;

  GstStructure* filter_caps_structure_;
  GstCaps* filter_caps_;
  GstStructure* encode_caps_structure_;
  GstCaps* encode_caps_;

  GstElement* pipeline_;
  GstBin* bin_;

  GstElement* source_;
  GstElement* source_filter_;
  GstElement* decode_;
  GstElement* convert_;
  GstElement* convert_queue_;
  GstElement* encode_;
  GstElement* encode_filter_;
  GstElement* encode_queue_;
  GstElement* payload_;
  GstElement* sink_;
};

#endif  // GSTREAMER_VIDEO_OUTPUT_PIPELINE_HPP
