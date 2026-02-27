#pragma once

#define H264_PAYLOAD_TYPE 102
#define VP9_PAYLOAD_TYPE 100
#define VP8_PAYLOAD_TYPE 96
#define X264_ENC_PRESET_ULTRAFAST 1
#define X264_ENC_TUNE_ZEROLATENCY 4
#define RTP_MTU 1200

#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/webrtc.h>
#include <memory>
#include <string>
#include "signaling_client.hpp"

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

  void process_signaling_messages();

  inline GstElement* get_sink() const { return sink_; }

  /**
   * Connect to WebRTC signaling server.
   * @param server_url WebSocket URL (e.g., "ws://192.168.1.100:8765")
   * @param room_id Room identifier (shared with receiver)
   * @return true if connection initiated
   */
  bool connect_signaling(const std::string& server_url, const std::string& room_id);

  /**
   * Begin the WebRTC negotiation process.
   * Should be called after both sender and receiver join the room.
   */
  void start_negotiation();

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

  static void on_negotiation_needed_cb(GstElement* webrtcbin, gpointer user_data);
  static void on_ice_candidate_cb(GstElement* webrtcbin, guint mlineindex, gchar* candidate, gpointer user_data);
  static void on_offer_created_cb(GstPromise* promise, gpointer user_data);
  
  // Signaling client callbacks
  void on_remote_sdp(const std::string& sdp);
  void on_remote_ice(int sdp_mline_index, const std::string& candidate);
  void on_signaling_connected();
  void on_signaling_error(const std::string& error);
  
  void apply_remote_sdp_answer(const std::string& answer_sdp);
  void add_remote_ice(guint mlineindex, const std::string& candidate);

  void send_sdp_to_peer(const std::string& sdp);
  void send_ice_to_peer(guint mlineindex, const std::string& candidate);
  std::string modify_ice_candidate(const std::string& candidate);

  std::unique_ptr<SignalingClient> signaling_client_;
  std::string external_ip_;  // For forcing specific IP in ICE candidates
};
