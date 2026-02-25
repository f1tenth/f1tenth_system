/*
 * Implementation of WebRTC signaling client.
 * Uses ixwebsocket library (pure C++, minimal dependencies).
 */

#include "signaling_client.hpp"
#include <iostream>
#include <nlohmann/json.hpp>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>

using json = nlohmann::json;

// ============================================================================
// WebSocketImpl: Concrete WebSocket implementation
// ============================================================================

class SignalingClient::WebSocketImpl {
public:
    ix::WebSocket ws;
    std::string server_url;
    std::string room_id;
    std::string role;
    
    // Callback fptr to SignalingClient (for routing)
    SignalingClient* parent{nullptr};
    std::mutex parent_mutex;
};

// ============================================================================
// SignalingClient Implementation
// ============================================================================

SignalingClient::SignalingClient()
    : ws_(std::make_unique<WebSocketImpl>()) {}

SignalingClient::~SignalingClient() {
    disconnect();
}

bool SignalingClient::connect(const std::string& server_url, const std::string& room_id, const std::string& role) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (connected_) {
        if (on_error_) on_error_("Already connected");
        return false;
    }
    
    room_id_ = room_id;
    role_ = role;
    
    // Setup WebSocket
    ws_->server_url = server_url;
    ws_->room_id = room_id;
    ws_->role = role;
    ws_->parent = this;
    
    // Setup callbacks
    ws_->ws.setUrl(server_url);
    
    ws_->ws.setOnMessageCallback([this](const ix::WebSocketMessagePtr& msg) {
        if (msg->type == ix::WebSocketMessageType::Message) {
            on_ws_message(msg->str);
        } else if (msg->type == ix::WebSocketMessageType::Open) {
            on_ws_open();
        } else if (msg->type == ix::WebSocketMessageType::Error) {
            on_ws_error(msg->errorInfo.reason);
        } else if (msg->type == ix::WebSocketMessageType::Close) {
            on_ws_close();
        }
    });
    
    // Start connection (asynchronous)
    ws_->ws.start();
    
    return true;
}

void SignalingClient::disconnect() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!connected_) return;
    
    // Send bye message if still connected
    try {
        json bye_msg;
        bye_msg["type"] = "bye";
        bye_msg["room"] = room_id_;
        bye_msg["role"] = role_;
        ws_->ws.send(bye_msg.dump());
    } catch (...) {}
    
    ws_->ws.stop();
    connected_ = false;
}

void SignalingClient::send_sdp(const std::string& sdp, const std::string& type) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!connected_) {
        if (on_error_) on_error_("Not connected");
        return;
    }
    
    try {
        json msg;
        msg["type"] = type;  // "offer" or "answer"
        msg["room"] = room_id_;
        msg["role"] = role_;
        msg["sdp"] = sdp;
        
        ws_->ws.send(msg.dump());
    } catch (const std::exception& e) {
        if (on_error_) on_error_(std::string("Failed to send SDP: ") + e.what());
    }
}

void SignalingClient::send_ice_candidate(int sdp_mline_index, const std::string& candidate) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!connected_) {
        return;  // Silent fail for ICE (candidates may be dropped)
    }
    
    try {
        json msg;
        msg["type"] = "ice";
        msg["room"] = room_id_;
        msg["role"] = role_;
        msg["sdpMLineIndex"] = sdp_mline_index;
        msg["candidate"] = candidate;
        
        ws_->ws.send(msg.dump());
    } catch (...) {
        // Silently drop failed ICE candidates
    }
}

void SignalingClient::send_bye() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!connected_) return;
    
    try {
        json msg;
        msg["type"] = "bye";
        msg["room"] = room_id_;
        msg["role"] = role_;
        ws_->ws.send(msg.dump());
    } catch (...) {}
}

void SignalingClient::process_pending_messages() {
    dispatch_pending_messages();
}

bool SignalingClient::is_connected() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return connected_;
}

// ============================================================================
// Private: WebSocket Event Handlers
// ============================================================================

void SignalingClient::on_ws_open() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        connected_ = true;
    }
    
    // Send join message
    try {
        json join_msg;
        join_msg["type"] = "join";
        join_msg["room"] = room_id_;
        join_msg["role"] = role_;
        ws_->ws.send(join_msg.dump());
    } catch (const std::exception& e) {
        enqueue_message({PendingMessage::ERROR, std::string("Failed to send join: ") + e.what()});
    }
    
    enqueue_message({PendingMessage::CONNECTED});
}

void SignalingClient::on_ws_message(const std::string& message) {
    try {
        json msg = json::parse(message);
        std::string type = msg.value("type", "");
        
        if (type == "peer-ready") {
            enqueue_message({PendingMessage::PEER_READY});
        }
        else if (type == "offer") {
            std::string sdp = msg.value("sdp", "");
            enqueue_message({PendingMessage::REMOTE_SDP, sdp});
        }
        else if (type == "answer") {
            std::string sdp = msg.value("sdp", "");
            enqueue_message({PendingMessage::REMOTE_SDP, sdp});
        }
        else if (type == "ice") {
            int mline = msg.value("sdpMLineIndex", 0);
            std::string candidate = msg.value("candidate", "");
            PendingMessage pm{PendingMessage::REMOTE_ICE, candidate};
            pm.data3 = mline;
            enqueue_message(pm);
        }
        else if (type == "error") {
            std::string error_msg = msg.value("message", "Unknown error");
            enqueue_message({PendingMessage::ERROR, error_msg});
        }
        else if (type == "peer-left") {
            enqueue_message({PendingMessage::ERROR, "Peer disconnected"});
        }
    } catch (const std::exception& e) {
        enqueue_message({PendingMessage::ERROR, std::string("JSON parse error: ") + e.what()});
    }
}

void SignalingClient::on_ws_error(const std::string& error) {
    enqueue_message({PendingMessage::ERROR, error});
}

void SignalingClient::on_ws_close() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        connected_ = false;
    }
    enqueue_message({PendingMessage::DISCONNECTED});
}

// ============================================================================
// Private: Message Queue Management
// ============================================================================

void SignalingClient::enqueue_message(const PendingMessage& msg) {
    std::lock_guard<std::mutex> lock(message_queue_mutex_);
    pending_messages_.push(msg);
}

void SignalingClient::dispatch_pending_messages() {
    std::queue<PendingMessage> to_dispatch;
    {
        std::lock_guard<std::mutex> lock(message_queue_mutex_);
        to_dispatch = pending_messages_;
        while (!pending_messages_.empty()) {
            pending_messages_.pop();
        }
    }
    
    while (!to_dispatch.empty()) {
        const auto& msg = to_dispatch.front();
        
        switch (msg.type) {
            case PendingMessage::PEER_READY:
                if (on_peer_ready_) on_peer_ready_();
                break;
            case PendingMessage::REMOTE_SDP:
                if (on_remote_sdp_) on_remote_sdp_(msg.data1);
                break;
            case PendingMessage::REMOTE_ICE:
                if (on_remote_ice_) on_remote_ice_(msg.data3, msg.data2);
                break;
            case PendingMessage::ERROR:
                if (on_error_) on_error_(msg.data1);
                break;
            case PendingMessage::CONNECTED:
                if (on_connected_) on_connected_();
                break;
            case PendingMessage::DISCONNECTED:
                if (on_disconnected_) on_disconnected_();
                break;
        }
        
        to_dispatch.pop();
    }
}
