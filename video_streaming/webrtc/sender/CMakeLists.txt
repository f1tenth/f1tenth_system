cmake_minimum_required(VERSION 3.10)
project(video_streaming_webrtc_sender LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-sdp-1.0 gstreamer-webrtc-1.0 gobject-2.0 glib-2.0)

add_executable(webrtc_sender
  webrtc_sender_main.cpp
  gstreamer_sender_webrtc.cpp
  signaling_client.cpp
)

target_include_directories(webrtc_sender PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GST_INCLUDE_DIRS}
)

target_compile_options(webrtc_sender PRIVATE ${GST_CFLAGS_OTHER})
target_link_directories(webrtc_sender PRIVATE ${GST_LIBRARY_DIRS})
target_link_libraries(webrtc_sender PRIVATE
  ${GST_LIBRARIES}
  ixwebsocket
  ssl
  crypto
  z
  pthread
)
