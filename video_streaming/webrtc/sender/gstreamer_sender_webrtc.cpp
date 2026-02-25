#include "gstreamer_sender_webrtc.hpp"
#include "signaling_client.hpp"

#include <iostream>
#include <utility>

static GstElement* create_video_convert(const gchar* name) {
  GstElement* video_convert = gst_element_factory_make("nvvidconv", name);

  if (!video_convert) {
    video_convert = gst_element_factory_make("videoconvert", name);
  }

  return video_convert;
}

static GstElement* create_h264_encoder(const gchar* name) {
  GstElement* encoder = gst_element_factory_make("nvv4l2h264enc", name);

  if (!encoder) {
    std::cout << "Using x264enc as a fallback" << std::endl;

    encoder = gst_element_factory_make("x264enc", name);
    g_object_set(encoder, "speed-preset", X264_ENC_PRESET_ULTRAFAST, NULL);
    g_object_set(encoder, "tune", X264_ENC_TUNE_ZEROLATENCY, NULL);
    return encoder;
  }

  std::cout << "Using nvv4l2h264enc" << std::endl;

  g_object_set(encoder, "preset-level", 2, NULL);
  g_object_set(encoder, "profile", 0, NULL);
  g_object_set(encoder, "insert-sps-pps", true, NULL);
  g_object_set(encoder, "poc-type", 2, NULL);
  g_object_set(encoder, "maxperf-enable", true, NULL);

  return encoder;
}

VideoOutputPipeline::VideoOutputPipeline(VideoOutputTrackInfo info)
    : info_(info),
      encode_caps_(gst_caps_new_empty()),
      pipeline_(gst_pipeline_new("pipeline")),
      bin_(GST_BIN(pipeline_)),
      convert_(create_video_convert("convert")),
      convert_queue_(gst_element_factory_make("queue", "convert_queue")),
      encode_filter_(gst_element_factory_make("capsfilter", "encode_filter")),
      encode_queue_(gst_element_factory_make("queue", "encode_queue")),
      sink_(gst_element_factory_make("webrtcbin", "sink")) {

  g_object_set(sink_, "stun-server", "stun://stun.l.google.com:19302", NULL);
  g_signal_connect(sink_, "on-negotiation-needed", G_CALLBACK(VideoOutputPipeline::on_negotiation_needed_cb), this);
  g_signal_connect(sink_, "on-ice-candidate", G_CALLBACK(VideoOutputPipeline::on_ice_candidate_cb), this);

  // Make the queues leaky to prevent stale data from being processed when the pipeline is not consuming data fast enough.
  g_object_set(convert_queue_, "max-size-buffers", 10, NULL);  // Keep only 3 frames
  g_object_set(convert_queue_, "leaky", 2, NULL);  // 2 = drop old frames

  g_object_set(encode_queue_, "max-size-buffers", 10, NULL);
  g_object_set(encode_queue_, "leaky", 2, NULL);  // Drop old encoded frames
  

  switch (info_.source) {
    case VideoSource::v4l2: {
      filter_caps_structure_ = gst_structure_new(
          info.media_type.c_str(), "width", G_TYPE_INT, info.caps.width,
          "height", G_TYPE_INT, info.caps.height, "framerate",
          GST_TYPE_FRACTION, info.caps.framerate.num, info.caps.framerate.den,
          NULL);
      filter_caps_ = gst_caps_new_full(filter_caps_structure_, NULL);
      source_ = gst_element_factory_make("v4l2src", "source");
      source_filter_ = gst_element_factory_make("capsfilter", "source_filter");

      g_object_set(source_, "device", info.device.c_str(), NULL);

      break;
    }

    case VideoSource::navargus: {
      filter_caps_ = gst_caps_from_string(
          "video/x-raw(memory:NVMM), width=240, height=120, framerate=30/1");
      source_ = gst_element_factory_make("nvarguscamerasrc", "source");
      source_filter_ = gst_element_factory_make("capsfilter", "source_filter");

      g_object_set(source_, "sensor_id", std::stoi(info.device), NULL);

      break;
    }

    case VideoSource::videotestsrc: {
      filter_caps_structure_ = gst_structure_new(
          info.media_type.c_str(), "width", G_TYPE_INT, info.caps.width,
          "height", G_TYPE_INT, info.caps.height, "framerate",
          GST_TYPE_FRACTION, info.caps.framerate.num, info.caps.framerate.den,
          NULL);
      filter_caps_ = gst_caps_new_full(filter_caps_structure_, NULL);
      source_ = gst_element_factory_make("videotestsrc", "source");
      source_filter_ = gst_element_factory_make("capsfilter", "source_filter");

      break;
    }
  }

  switch (info_.codec) {
    case VideoCodec::H264: {
      encode_caps_structure_ = gst_structure_from_string(
          "video/x-h264, profile=(string)constrained-baseline", NULL);
      gst_caps_append_structure(encode_caps_, encode_caps_structure_);
      encode_ = create_h264_encoder("encode");
      payload_ = gst_element_factory_make("rtph264pay", "pay");

      g_object_set(encode_filter_, "caps", encode_caps_, NULL);
      g_object_set(payload_, "pt", H264_PAYLOAD_TYPE, NULL);

      break;
    }

    case VideoCodec::VP8:

    default: {
      encode_caps_structure_ = gst_structure_from_string("video/x-vp8", NULL);
      gst_caps_append_structure(encode_caps_, encode_caps_structure_);
      encode_ = gst_element_factory_make("vp8enc", "encode");
      payload_ = gst_element_factory_make("rtpvp8pay", "pay");

      g_object_set(encode_, "deadline", 1, NULL);
      g_object_set(encode_filter_, "caps", encode_caps_, NULL);
      g_object_set(payload_, "pt", VP8_PAYLOAD_TYPE, NULL);

      break;
    }
  }

  gst_bin_add_many(bin_, source_, source_filter_, NULL);
  gst_element_link_many(source_, source_filter_, NULL);

  if (!info.media_type.rfind("image/jpeg", 0)) {
    GstElement* decode = gst_element_factory_make("jpegdec", "decode");
    GstElement* decode_queue =
        gst_element_factory_make("queue", "decode_queue");
    gst_bin_add_many(bin_, decode, decode_queue, convert_, NULL);
    gst_element_link_many(source_filter_, decode, decode_queue, convert_, NULL);
  } else {
    gst_bin_add_many(bin_, convert_, NULL);
    gst_element_link(source_filter_, convert_);
  }

  g_object_set(source_filter_, "caps", filter_caps_, NULL);
  g_object_set(payload_, "mtu", RTP_MTU, NULL);

  gst_bin_add_many(bin_, convert_queue_, encode_, encode_filter_, encode_queue_, payload_, sink_, NULL);

  // Normal linking up to payloader
  if (!gst_element_link_many(convert_, convert_queue_, encode_, encode_filter_, encode_queue_, payload_, NULL)) {
    std::cerr << "Failed to link elements up to payload" << std::endl;
  }

  // Request webrtcbin sink pad and link manually
  GstPad* src_pad = gst_element_get_static_pad(payload_, "src");
  GstPad* webrtc_sink_pad = gst_element_request_pad_simple(sink_, "sink_%u");

  if (!src_pad || !webrtc_sink_pad) {
    std::cerr << "Failed to get pads for payload->webrtcbin link" << std::endl;
  } else {
    GstPadLinkReturn link_ret = gst_pad_link(src_pad, webrtc_sink_pad);
    if (link_ret != GST_PAD_LINK_OK) {
      std::cerr << "Failed to pad-link payload to webrtcbin: " << link_ret << std::endl;
    }
  }
  if (src_pad) gst_object_unref(src_pad);
  if (webrtc_sink_pad) gst_object_unref(webrtc_sink_pad);

  // Initialize signaling client (not connected until connect_signaling is called)
  signaling_client_ = std::make_unique<SignalingClient>();
}

VideoOutputPipeline::~VideoOutputPipeline() {
  stop();
  gst_object_unref(pipeline_);
  gst_caps_unref(filter_caps_);
  gst_caps_unref(encode_caps_);
}

void VideoOutputPipeline::start() {
  GstStateChangeReturn ret =
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret != GST_STATE_CHANGE_FAILURE) {
    std::cout << "Successfully turned ON VIDEO pipeline" << std::endl;
  } else {
    std::cerr << "Failed to turn ON VIDEO pipeline" << std::endl;
  }
}

void VideoOutputPipeline::stop() {
  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_NULL);
  if (ret != GST_STATE_CHANGE_FAILURE) {
    std::cout << "Successfully turned OFF VIDEO pipeline" << std::endl;
  } else {
    std::cerr << "Failed to turn OFF VIDEO pipeline" << std::endl;
  }
}

void VideoOutputPipeline::process_signaling_messages() {
  if (signaling_client_) {
    signaling_client_->process_pending_messages();
  }
}

void VideoOutputPipeline::on_negotiation_needed_cb(GstElement* webrtcbin, gpointer user_data) {
  auto* self = static_cast<VideoOutputPipeline*>(user_data);
  GstPromise* promise = gst_promise_new_with_change_func(
      VideoOutputPipeline::on_offer_created_cb, self, NULL);
  g_signal_emit_by_name(webrtcbin, "create-offer", NULL, promise);
}

void VideoOutputPipeline::on_offer_created_cb(GstPromise* promise, gpointer user_data) {
  auto* self = static_cast<VideoOutputPipeline*>(user_data);

  const GstStructure* reply = gst_promise_get_reply(promise);
  GstWebRTCSessionDescription* offer = NULL;
  gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, NULL);
  gst_promise_unref(promise);

  GstPromise* local_desc_promise = gst_promise_new();
  g_signal_emit_by_name(self->get_sink(), "set-local-description", offer, local_desc_promise);
  gst_promise_interrupt(local_desc_promise);
  gst_promise_unref(local_desc_promise);

  gchar* sdp_text = gst_sdp_message_as_text(offer->sdp);
  self->send_sdp_to_peer(sdp_text);
  g_free(sdp_text);

  gst_webrtc_session_description_free(offer);
}

void VideoOutputPipeline::on_ice_candidate_cb(
    GstElement* /*webrtcbin*/, guint mlineindex, gchar* candidate, gpointer user_data) {
  auto* self = static_cast<VideoOutputPipeline*>(user_data);
  self->send_ice_to_peer(mlineindex, candidate ? candidate : "");
}

void VideoOutputPipeline::apply_remote_sdp_answer(const std::string& answer_sdp) {
  GstSDPMessage* sdp = NULL;
  gst_sdp_message_new(&sdp);

  if (gst_sdp_message_parse_buffer(
          reinterpret_cast<const guint8*>(answer_sdp.c_str()),
          answer_sdp.size(), sdp) != GST_SDP_OK) {
    std::cerr << "Failed to parse remote SDP answer" << std::endl;
    gst_sdp_message_free(sdp);
    return;
  }

  GstWebRTCSessionDescription* answer =
      gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp);

  GstPromise* promise = gst_promise_new();
  g_signal_emit_by_name(sink_, "set-remote-description", answer, promise);
  gst_promise_interrupt(promise);
  gst_promise_unref(promise);

  gst_webrtc_session_description_free(answer);
}

void VideoOutputPipeline::add_remote_ice(guint mlineindex, const std::string& candidate) {
  g_signal_emit_by_name(sink_, "add-ice-candidate", mlineindex, candidate.c_str());
}

void VideoOutputPipeline::send_sdp_to_peer(const std::string& sdp) {
  if (signaling_client_) {
    signaling_client_->send_sdp(sdp, "offer");
  } else {
    std::cerr << "Signaling client not initialized" << std::endl;
  }
}

void VideoOutputPipeline::send_ice_to_peer(guint mlineindex, const std::string& candidate) {
  if (signaling_client_) {
    signaling_client_->send_ice_candidate(static_cast<int>(mlineindex), candidate);
  }
}

bool VideoOutputPipeline::connect_signaling(const std::string& server_url, const std::string& room_id) {
  if (!signaling_client_) {
    std::cerr << "Signaling client not initialized" << std::endl;
    return false;
  }

  // Setup callbacks
  signaling_client_->set_on_remote_sdp([this](const std::string& sdp) {
    this->on_remote_sdp(sdp);
  });

  signaling_client_->set_on_remote_ice([this](int mline, const std::string& candidate) {
    this->on_remote_ice(mline, candidate);
  });

  signaling_client_->set_on_connected([this]() {
    this->on_signaling_connected();
  });

  signaling_client_->set_on_error([this](const std::string& error) {
    this->on_signaling_error(error);
  });

  return signaling_client_->connect(server_url, room_id, "sender");
}

void VideoOutputPipeline::on_remote_sdp(const std::string& sdp) {
  std::cout << "[SIGNALING] Received remote SDP (" << sdp.size() << " bytes)" << std::endl;
  apply_remote_sdp_answer(sdp);
}

void VideoOutputPipeline::on_remote_ice(int sdp_mline_index, const std::string& candidate) {
  std::cout << "[SIGNALING] Received remote ICE candidate (mline " << sdp_mline_index << ")" << std::endl;
  add_remote_ice(static_cast<guint>(sdp_mline_index), candidate);
}

void VideoOutputPipeline::on_signaling_connected() {
  std::cout << "[SIGNALING] Connected to server" << std::endl;
}

void VideoOutputPipeline::on_signaling_error(const std::string& error) {
  std::cerr << "[SIGNALING] Error: " << error << std::endl;
}