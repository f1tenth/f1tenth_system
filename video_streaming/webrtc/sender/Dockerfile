FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    git \
    ca-certificates \
    libssl-dev \
    zlib1g-dev \
    nlohmann-json3-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libglib2.0-dev \
    && rm -rf /var/lib/apt/lists/*

ARG IXWS_REF=v11.4.5

RUN git clone --depth 1 https://github.com/machinezone/IXWebSocket.git /tmp/IXWebSocket \
    && cd /tmp/IXWebSocket \
    && (git fetch --depth 1 origin "refs/tags/${IXWS_REF}:refs/tags/${IXWS_REF}" \
        && git checkout "${IXWS_REF}" \
        || echo "Warning: could not checkout ${IXWS_REF}, using default branch") \
    && cmake -S /tmp/IXWebSocket -B /tmp/IXWebSocket/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/IXWebSocket/build -j"$(nproc)" \
    && cmake --install /tmp/IXWebSocket/build \
    && rm -rf /tmp/IXWebSocket

WORKDIR /src
COPY . /src

RUN cmake -S . -B build && cmake --build build -j"$(nproc)"

FROM ubuntu:22.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 \
    libssl3 \
    zlib1g \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    libgstreamer-plugins-bad1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-nice \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local/lib/libixwebsocket.so* /usr/local/lib/
COPY --from=builder /src/build/webrtc_sender /usr/local/bin/webrtc_sender

RUN ldconfig

ENTRYPOINT ["/usr/local/bin/webrtc_sender"]
CMD ["--server-url", "ws://host.docker.internal:8765", "--room-id", "f1tenth", "--source", "videotestsrc"]
