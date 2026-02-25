cmake_minimum_required(VERSION 3.10)
project(video_streaming_rtp_sender LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(rtp_sender
  rtp_sender_main.cpp
  gstreamer_sender_rtp.cpp
)

target_include_directories(rtp_sender PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

find_package(PkgConfig)

if(PkgConfig_FOUND)
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gobject-2.0 glib-2.0)
  target_include_directories(rtp_sender PRIVATE ${GSTREAMER_INCLUDE_DIRS})
  target_compile_options(rtp_sender PRIVATE ${GSTREAMER_CFLAGS_OTHER})
  target_link_directories(rtp_sender PRIVATE ${GSTREAMER_LIBRARY_DIRS})
  target_link_libraries(rtp_sender PRIVATE ${GSTREAMER_LIBRARIES})
else()
  set(GSTREAMER_ROOT "$ENV{GSTREAMER_1_0_ROOT_MSVC_X86_64}")
  if(NOT GSTREAMER_ROOT)
    set(GSTREAMER_ROOT "$ENV{GSTREAMER_1_0_ROOT_X86_64}")
  endif()

  if(NOT GSTREAMER_ROOT)
    message(FATAL_ERROR
      "Could not find GStreamer. Install pkg-config (Linux) or set GSTREAMER_1_0_ROOT_MSVC_X86_64 / GSTREAMER_1_0_ROOT_X86_64 (Windows).")
  endif()

  target_include_directories(rtp_sender PRIVATE
    "${GSTREAMER_ROOT}/include/gstreamer-1.0"
    "${GSTREAMER_ROOT}/include/glib-2.0"
    "${GSTREAMER_ROOT}/lib/glib-2.0/include"
  )
  target_link_directories(rtp_sender PRIVATE "${GSTREAMER_ROOT}/lib")
  target_link_libraries(rtp_sender PRIVATE gstreamer-1.0 gstbase-1.0 gobject-2.0 glib-2.0)
endif()
