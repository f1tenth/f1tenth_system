/*
 * WebRTC signaling client for Jetson sender (C++ via WebSocket).
 * Handles connection to signaling server, SDP/ICE relay, and event callbacks.
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <thread>
#include <mutex>

/**
 * WebRTC signaling client interface.
 * Manages WebSocket connection to signaling server and routes SDP/ICE messages.
 * 
 * Typical flow:
 * 1. Create SignalingClient instance
 * 2. Set callbacks: on_local_sdp_needed, on_ice_candidate, on_remote_ice, etc.
 * 3. Call connect(server_url, room_id)
 * 4. When local SDP generated: call send_local_sdp(offer)
 * 5. When local ICE candidate generated: call send_ice_candidate(mlineindex, candidate)
 * 6. Callbacks fire when remote answer/ICE received
 */
class SignalingClient {
public:
    // ========== Callback Signatures ==========
    
    /// Called when peer is ready and negotiation should begin
    using OnPeerReadyCallback = std::function<void()>;
    
    /// Called when remote SDP answer is received
    using OnRemoteSdpCallback = std::function<void(const std::string& sdp)>;
    
    /// Called when remote ICE candidate is received
    using OnRemoteIceCallback = std::function<void(int sdp_mline_index, const std::string& candidate)>;
    
    /// Called on connection errors
    using OnErrorCallback = std::function<void(const std::string& error_message)>;
    
    /// Called when connection is established
    using OnConnectedCallback = std::function<void()>;
    
    /// Called when connection is closed
    using OnDisconnectedCallback = std::function<void()>;
    
    // ========== Public Methods ==========
    
    SignalingClient();
    ~SignalingClient();
    
    /**
     * Connect to signaling server.
     * @param server_url WebSocket URL (e.g., "ws://192.168.1.100:8765")
     * @param room_id Room identifier (shared between sender and receiver)
     * @param role "sender" or "receiver"
     * @return true if connection initiated (callbacks will fire async)
     */
    bool connect(const std::string& server_url, const std::string& room_id, const std::string& role = "sender");
    
    /**
     * Disconnect from server and clean up.
     */
    void disconnect();
    
    /**
     * Send local SDP offer/answer to peer via server.
     * @param sdp SDP content (text format)
     * @param type "offer" or "answer"
     */
    void send_sdp(const std::string& sdp, const std::string& type = "offer");
    
    /**
     * Send local ICE candidate to peer.
     * @param sdp_mline_index Media line index (usually 0 for single video stream)
     * @param candidate ICE candidate string
     */
    void send_ice_candidate(int sdp_mline_index, const std::string& candidate);
    
    /**
     * Signal peer disconnect.
     */
    void send_bye();
    
    // ========== Callback Registration ==========
    
    void set_on_peer_ready(OnPeerReadyCallback cb) { on_peer_ready_ = cb; }
    void set_on_remote_sdp(OnRemoteSdpCallback cb) { on_remote_sdp_ = cb; }
    void set_on_remote_ice(OnRemoteIceCallback cb) { on_remote_ice_ = cb; }
    void set_on_error(OnErrorCallback cb) { on_error_ = cb; }
    void set_on_connected(OnConnectedCallback cb) { on_connected_ = cb; }
    void set_on_disconnected(OnDisconnectedCallback cb) { on_disconnected_ = cb; }
    
    /**
     * Process pending messages (call from main loop or GStreamer event handler).
     * Allows async callbacks to be dispatched on your thread.
     */
    void process_pending_messages();
    
    /// Check if currently connected
    bool is_connected() const;

private:
    // ========== WebSocket Handling (Platform-Specific) ==========
    
    class WebSocketImpl;
    std::unique_ptr<WebSocketImpl> ws_;
    
    // ========== State and Callbacks ==========
    
    mutable std::mutex state_mutex_;
    bool connected_{false};
    std::string room_id_;
    std::string role_;
    
    OnPeerReadyCallback on_peer_ready_;
    OnRemoteSdpCallback on_remote_sdp_;
    OnRemoteIceCallback on_remote_ice_;
    OnErrorCallback on_error_;
    OnConnectedCallback on_connected_;
    OnDisconnectedCallback on_disconnected_;
    
    // ========== Message Queue (Thread-Safe) ==========
    
    struct PendingMessage {
        enum Type { PEER_READY, REMOTE_SDP, REMOTE_ICE, ERROR, CONNECTED, DISCONNECTED } type;
        std::string data1;  // SDP or error message
        std::string data2;  // candidate
        int data3{0};       // mline index
    };
    
    std::queue<PendingMessage> pending_messages_;
    mutable std::mutex message_queue_mutex_;
    
    // ========== Internal Handlers ==========
    
    void on_ws_open();
    void on_ws_message(const std::string& message);
    void on_ws_error(const std::string& error);
    void on_ws_close();
    
    void enqueue_message(const PendingMessage& msg);
    void dispatch_pending_messages();
};
