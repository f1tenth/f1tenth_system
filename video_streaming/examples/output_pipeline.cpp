#include "gstreamer/video/output_pipeline.hpp"

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

  g_object_set(encoder, "preset_level", 2, NULL);
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
      sink_(gst_element_factory_make("appsink", "sink")) {
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
  g_object_set(sink_, "sync", false, NULL);

  gst_bin_add_many(bin_, convert_queue_, encode_, encode_filter_, encode_queue_,
                   payload_, sink_, NULL);
  gst_element_link_many(convert_, convert_queue_, encode_, encode_filter_,
                        encode_queue_, payload_, sink_, NULL);
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
