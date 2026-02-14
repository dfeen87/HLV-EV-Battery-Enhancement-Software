/*
 * ============================================================================
 * HLV REST API SERVER v1.0
 * ============================================================================
 *
 * Lightweight HTTP/JSON REST API for HLV Battery Management observability.
 * Provides read-only endpoints for monitoring battery state, torque limits,
 * regen status, diagnostics, and energy metrics.
 *
 * DESIGN PRINCIPLES:
 *   - Read-only (GET endpoints only) for automotive safety
 *   - Thread-safe with mutex-protected shared data access
 *   - Non-blocking: runs in dedicated thread
 *   - Lightweight: POSIX sockets, no external HTTP libraries
 *   - JSON responses for easy integration
 *
 * ENDPOINTS:
 *   GET /health              - Server health check
 *   GET /api/battery         - Battery state (Ψ, Φ)
 *   GET /api/torque          - Torque limits and factors
 *   GET /api/regen           - Regen limits and constraints
 *   GET /api/limits          - All system limits
 *   GET /api/diagnostics     - Pack diagnostics
 *   GET /api/energy_cycle    - Energy cycle metrics
 *
 * USAGE:
 *   HLVRestAPIServer server(8080);
 *   server.start();
 *   server.update_state(enhanced_state, torque_result, regen_result);
 *   // ... server runs in background thread ...
 *   server.stop();
 *
 * AUTHORS: Don Michael Feeney Jr. & Claude (Anthropic)
 * DATE: February 2026
 * LICENSE: MIT
 * VERSION: 1.0.0
 *
 * ============================================================================
 */

#ifndef HLV_REST_API_SERVER_HPP
#define HLV_REST_API_SERVER_HPP

#include "hlv_battery_enhancement.hpp"
#include "hlv_bms_middleware_v2.hpp"
#include "torque_enhancement.hpp"
#include "hlv_regen_braking_manager_v1.hpp"
#include "hlv_energy_telemetry.hpp"

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <vector>
#include <memory>

// POSIX socket headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

namespace hlv {
namespace api {

// ============================================================================
// VERSION
// ============================================================================

constexpr int REST_API_VERSION_MAJOR = 1;
constexpr int REST_API_VERSION_MINOR = 0;
constexpr int REST_API_VERSION_PATCH = 0;

inline std::string get_rest_api_version() {
    return std::to_string(REST_API_VERSION_MAJOR) + "." +
           std::to_string(REST_API_VERSION_MINOR) + "." +
           std::to_string(REST_API_VERSION_PATCH);
}

// ============================================================================
// THREAD-SAFE STATE SNAPSHOT
// ============================================================================

struct APIStateSnapshot {
    // Battery state
    hlv::EnhancedState enhanced_state;
    hlv_plugin::DiagnosticReport diagnostics;
    
    // Torque state
    bool has_torque_data = false;
    double max_drive_torque_nm = 0.0;
    double max_regen_torque_nm = 0.0;
    std::string torque_limiting_factor;
    double torque_power_limit_kw = 0.0;
    
    // Regen state
    bool has_regen_data = false;
    double regen_max_torque_nm = 0.0;
    double regen_fraction = 0.0;
    std::string regen_limiting_factor;
    
    // Energy metrics
    bool has_energy_data = false;
    double energy_drive_kwh = 0.0;
    double energy_regen_kwh = 0.0;
    double energy_loss_kwh = 0.0;
    double delta_e_info_j = 0.0;
    double total_efficiency = 0.0;
    
    // Timestamp
    double timestamp_s = 0.0;
};

// ============================================================================
// JSON SERIALIZATION HELPERS
// ============================================================================

class JSONBuilder {
public:
    static std::string escape(const std::string& str) {
        std::ostringstream oss;
        for (char c : str) {
            switch (c) {
                case '"': oss << "\\\""; break;
                case '\\': oss << "\\\\"; break;
                case '\b': oss << "\\b"; break;
                case '\f': oss << "\\f"; break;
                case '\n': oss << "\\n"; break;
                case '\r': oss << "\\r"; break;
                case '\t': oss << "\\t"; break;
                default: oss << c; break;
            }
        }
        return oss.str();
    }
    
    static std::string to_json(double val, int precision = 6) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << val;
        return oss.str();
    }
    
    static std::string to_json(bool val) {
        return val ? "true" : "false";
    }
    
    static std::string to_json(int val) {
        return std::to_string(val);
    }
    
    static std::string to_json(const std::string& val) {
        return "\"" + escape(val) + "\"";
    }
};

// ============================================================================
// HTTP RESPONSE BUILDER
// ============================================================================

class HTTPResponse {
public:
    static std::string build_response(int status_code, 
                                      const std::string& status_text,
                                      const std::string& body,
                                      const std::string& content_type = "application/json") {
        std::ostringstream response;
        response << "HTTP/1.1 " << status_code << " " << status_text << "\r\n";
        response << "Content-Type: " << content_type << "\r\n";
        response << "Content-Length: " << body.length() << "\r\n";
        response << "Connection: close\r\n";
        response << "Access-Control-Allow-Origin: *\r\n";
        response << "\r\n";
        response << body;
        return response.str();
    }
    
    static std::string ok(const std::string& json_body) {
        return build_response(200, "OK", json_body);
    }
    
    static std::string not_found() {
        std::string body = "{\"error\":\"Endpoint not found\"}";
        return build_response(404, "Not Found", body);
    }
    
    static std::string method_not_allowed() {
        std::string body = "{\"error\":\"Only GET requests allowed\"}";
        return build_response(405, "Method Not Allowed", body);
    }
    
    static std::string internal_error(const std::string& message) {
        std::string body = "{\"error\":\"" + JSONBuilder::escape(message) + "\"}";
        return build_response(500, "Internal Server Error", body);
    }
};

// ============================================================================
// REST API SERVER
// ============================================================================

class HLVRestAPIServer {
private:
    int port_;
    int server_socket_;
    std::atomic<bool> running_;
    std::thread server_thread_;
    
    // Thread-safe state
    std::mutex state_mutex_;
    APIStateSnapshot current_state_;
    
    // Parse HTTP request to extract method and path
    bool parse_request(const std::string& request, std::string& method, std::string& path) {
        size_t first_space = request.find(' ');
        if (first_space == std::string::npos) return false;
        
        size_t second_space = request.find(' ', first_space + 1);
        if (second_space == std::string::npos) return false;
        
        method = request.substr(0, first_space);
        path = request.substr(first_space + 1, second_space - first_space - 1);
        
        // Remove query parameters if present
        size_t query_pos = path.find('?');
        if (query_pos != std::string::npos) {
            path = path.substr(0, query_pos);
        }
        
        return true;
    }
    
    // Generate JSON for /health endpoint
    std::string handle_health() {
        std::ostringstream json;
        json << "{\n";
        json << "  \"status\": \"healthy\",\n";
        json << "  \"version\": \"" << get_rest_api_version() << "\",\n";
        json << "  \"service\": \"HLV Battery Management REST API\",\n";
        json << "  \"timestamp\": " << JSONBuilder::to_json(current_state_.timestamp_s) << "\n";
        json << "}";
        return json.str();
    }
    
    // Generate JSON for /api/battery endpoint
    std::string handle_battery() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto& s = current_state_.enhanced_state.state;
        
        std::ostringstream json;
        json << "{\n";
        json << "  \"physical_state\": {\n";
        json << "    \"voltage_v\": " << JSONBuilder::to_json(s.voltage) << ",\n";
        json << "    \"current_a\": " << JSONBuilder::to_json(s.current) << ",\n";
        json << "    \"temperature_c\": " << JSONBuilder::to_json(s.temperature) << ",\n";
        json << "    \"state_of_charge\": " << JSONBuilder::to_json(s.state_of_charge) << ",\n";
        json << "    \"energy_psi_j\": " << JSONBuilder::to_json(s.energy_psi, 2) << "\n";
        json << "  },\n";
        json << "  \"informational_state\": {\n";
        json << "    \"entropy\": " << JSONBuilder::to_json(s.entropy) << ",\n";
        json << "    \"cycle_count\": " << JSONBuilder::to_json(s.cycle_count) << ",\n";
        json << "    \"degradation\": " << JSONBuilder::to_json(s.degradation) << ",\n";
        json << "    \"phi_magnitude\": " << JSONBuilder::to_json(s.phi_magnitude) << ",\n";
        json << "    \"energy_phi_j\": " << JSONBuilder::to_json(s.energy_phi, 2) << "\n";
        json << "  },\n";
        json << "  \"hlv_metrics\": {\n";
        json << "    \"lambda\": " << JSONBuilder::to_json(s.lambda, 9) << ",\n";
        json << "    \"metric_trace\": " << JSONBuilder::to_json(s.g_eff.trace()) << ",\n";
        json << "    \"energy_metric_j\": " << JSONBuilder::to_json(s.energy_metric, 2) << ",\n";
        json << "    \"energy_total_j\": " << JSONBuilder::to_json(s.energy_total, 2) << "\n";
        json << "  },\n";
        json << "  \"health\": {\n";
        json << "    \"remaining_capacity_percent\": " << JSONBuilder::to_json(current_state_.enhanced_state.health.remaining_capacity_percent) << ",\n";
        json << "    \"cycles_to_80_percent\": " << JSONBuilder::to_json(current_state_.enhanced_state.health.cycles_to_80_percent) << ",\n";
        json << "    \"confidence\": " << JSONBuilder::to_json(current_state_.enhanced_state.health.confidence) << ",\n";
        json << "    \"warning_triggered\": " << JSONBuilder::to_json(current_state_.enhanced_state.health.warning_triggered) << "\n";
        json << "  },\n";
        json << "  \"timestamp_s\": " << JSONBuilder::to_json(current_state_.timestamp_s) << "\n";
        json << "}";
        return json.str();
    }
    
    // Generate JSON for /api/torque endpoint
    std::string handle_torque() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        std::ostringstream json;
        json << "{\n";
        json << "  \"available\": " << JSONBuilder::to_json(current_state_.has_torque_data) << ",\n";
        if (current_state_.has_torque_data) {
            json << "  \"max_drive_torque_nm\": " << JSONBuilder::to_json(current_state_.max_drive_torque_nm) << ",\n";
            json << "  \"max_regen_torque_nm\": " << JSONBuilder::to_json(current_state_.max_regen_torque_nm) << ",\n";
            json << "  \"power_limit_kw\": " << JSONBuilder::to_json(current_state_.torque_power_limit_kw) << ",\n";
            json << "  \"limiting_factor\": " << JSONBuilder::to_json(current_state_.torque_limiting_factor) << ",\n";
        }
        json << "  \"timestamp_s\": " << JSONBuilder::to_json(current_state_.timestamp_s) << "\n";
        json << "}";
        return json.str();
    }
    
    // Generate JSON for /api/regen endpoint
    std::string handle_regen() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        std::ostringstream json;
        json << "{\n";
        json << "  \"available\": " << JSONBuilder::to_json(current_state_.has_regen_data) << ",\n";
        if (current_state_.has_regen_data) {
            json << "  \"max_regen_torque_nm\": " << JSONBuilder::to_json(current_state_.regen_max_torque_nm) << ",\n";
            json << "  \"regen_fraction\": " << JSONBuilder::to_json(current_state_.regen_fraction) << ",\n";
            json << "  \"limiting_factor\": " << JSONBuilder::to_json(current_state_.regen_limiting_factor) << ",\n";
            json << "  \"charge_acceptance\": {\n";
            json << "    \"soc\": " << JSONBuilder::to_json(current_state_.enhanced_state.state.state_of_charge) << ",\n";
            json << "    \"temperature_c\": " << JSONBuilder::to_json(current_state_.enhanced_state.state.temperature) << ",\n";
            json << "    \"voltage_v\": " << JSONBuilder::to_json(current_state_.enhanced_state.state.voltage) << "\n";
            json << "  },\n";
        }
        json << "  \"timestamp_s\": " << JSONBuilder::to_json(current_state_.timestamp_s) << "\n";
        json << "}";
        return json.str();
    }
    
    // Generate JSON for /api/limits endpoint
    std::string handle_limits() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto& diag = current_state_.diagnostics;
        
        std::ostringstream json;
        json << "{\n";
        json << "  \"battery\": {\n";
        json << "    \"min_soc\": 0.05,\n";
        json << "    \"max_soc\": 0.95,\n";
        json << "    \"min_voltage_v\": " << JSONBuilder::to_json(current_state_.diagnostics.min_cell_voltage * 96) << ",\n";
        json << "    \"max_voltage_v\": " << JSONBuilder::to_json(current_state_.diagnostics.max_cell_voltage * 96) << ",\n";
        json << "    \"min_temp_c\": -20.0,\n";
        json << "    \"max_temp_c\": 60.0\n";
        json << "  },\n";
        json << "  \"power\": {\n";
        if (current_state_.has_torque_data) {
            json << "    \"max_discharge_kw\": " << JSONBuilder::to_json(current_state_.torque_power_limit_kw) << ",\n";
        } else {
            json << "    \"max_discharge_kw\": 250.0,\n";
        }
        json << "    \"max_charge_kw\": 120.0\n";
        json << "  },\n";
        json << "  \"torque\": {\n";
        if (current_state_.has_torque_data) {
            json << "    \"max_drive_nm\": " << JSONBuilder::to_json(current_state_.max_drive_torque_nm) << ",\n";
            json << "    \"max_regen_nm\": " << JSONBuilder::to_json(current_state_.max_regen_torque_nm) << "\n";
        } else {
            json << "    \"max_drive_nm\": 400.0,\n";
            json << "    \"max_regen_nm\": 250.0\n";
        }
        json << "  },\n";
        json << "  \"timestamp_s\": " << JSONBuilder::to_json(current_state_.timestamp_s) << "\n";
        json << "}";
        return json.str();
    }
    
    // Generate JSON for /api/diagnostics endpoint
    std::string handle_diagnostics() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto& diag = current_state_.diagnostics;
        
        std::ostringstream json;
        json << "{\n";
        json << "  \"pack\": {\n";
        json << "    \"soh_percent\": " << JSONBuilder::to_json(diag.pack_soh_percent) << ",\n";
        json << "    \"health_percent\": " << JSONBuilder::to_json(diag.pack_health_percent) << ",\n";
        json << "    \"total_cells\": " << JSONBuilder::to_json(diag.total_cells) << ",\n";
        json << "    \"weak_cell_count\": " << JSONBuilder::to_json(diag.weak_cell_count) << "\n";
        json << "  },\n";
        json << "  \"voltage\": {\n";
        json << "    \"min_cell_v\": " << JSONBuilder::to_json(diag.min_cell_voltage) << ",\n";
        json << "    \"max_cell_v\": " << JSONBuilder::to_json(diag.max_cell_voltage) << ",\n";
        json << "    \"imbalance_mv\": " << JSONBuilder::to_json(diag.voltage_imbalance_mv) << ",\n";
        json << "    \"voltage_warning\": " << JSONBuilder::to_json(diag.voltage_warning) << "\n";
        json << "  },\n";
        json << "  \"thermal\": {\n";
        json << "    \"average_temp_c\": " << JSONBuilder::to_json(diag.average_cell_temp_c) << ",\n";
        json << "    \"max_temp_c\": " << JSONBuilder::to_json(diag.max_cell_temp_c) << ",\n";
        json << "    \"min_temp_c\": " << JSONBuilder::to_json(diag.min_cell_temp_c) << ",\n";
        json << "    \"thermal_spread_c\": " << JSONBuilder::to_json(diag.max_cell_temp_c - diag.min_cell_temp_c) << ",\n";
        json << "    \"thermal_warning\": " << JSONBuilder::to_json(diag.thermal_warning) << "\n";
        json << "  },\n";
        json << "  \"hlv_metrics\": {\n";
        json << "    \"metric_trace\": " << JSONBuilder::to_json(diag.metric_trace) << ",\n";
        json << "    \"entropy\": " << JSONBuilder::to_json(diag.entropy_level) << ",\n";
        json << "    \"phi_magnitude\": " << JSONBuilder::to_json(diag.phi_magnitude) << ",\n";
        json << "    \"confidence\": " << JSONBuilder::to_json(diag.hlv_confidence) << "\n";
        json << "  },\n";
        json << "  \"warnings\": {\n";
        json << "    \"weak_cell\": " << JSONBuilder::to_json(diag.weak_cell_warning) << ",\n";
        json << "    \"thermal\": " << JSONBuilder::to_json(diag.thermal_warning) << ",\n";
        json << "    \"voltage\": " << JSONBuilder::to_json(diag.voltage_warning) << ",\n";
        json << "    \"degradation\": " << JSONBuilder::to_json(diag.degradation_warning) << ",\n";
        json << "    \"balancing_required\": " << JSONBuilder::to_json(diag.balancing_required) << ",\n";
        json << "    \"safety_fault\": " << JSONBuilder::to_json(diag.safety_fault) << "\n";
        json << "  },\n";
        json << "  \"timestamp_s\": " << JSONBuilder::to_json(current_state_.timestamp_s) << "\n";
        json << "}";
        return json.str();
    }
    
    // Generate JSON for /api/energy_cycle endpoint
    std::string handle_energy_cycle() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        std::ostringstream json;
        json << "{\n";
        json << "  \"available\": " << JSONBuilder::to_json(current_state_.has_energy_data) << ",\n";
        if (current_state_.has_energy_data) {
            json << "  \"energy_drive_kwh\": " << JSONBuilder::to_json(current_state_.energy_drive_kwh) << ",\n";
            json << "  \"energy_regen_kwh\": " << JSONBuilder::to_json(current_state_.energy_regen_kwh) << ",\n";
            json << "  \"energy_loss_kwh\": " << JSONBuilder::to_json(current_state_.energy_loss_kwh) << ",\n";
            json << "  \"delta_e_info_j\": " << JSONBuilder::to_json(current_state_.delta_e_info_j, 2) << ",\n";
            json << "  \"total_efficiency\": " << JSONBuilder::to_json(current_state_.total_efficiency) << ",\n";
        }
        json << "  \"cycle_metrics\": {\n";
        json << "    \"cycle_count\": " << JSONBuilder::to_json(current_state_.enhanced_state.state.cycle_count) << ",\n";
        json << "    \"charge_throughput_ah\": " << JSONBuilder::to_json(current_state_.enhanced_state.state.charge_throughput_ah) << ",\n";
        json << "    \"capacity_fade\": " << JSONBuilder::to_json(current_state_.enhanced_state.state.capacity_fade) << "\n";
        json << "  },\n";
        json << "  \"timestamp_s\": " << JSONBuilder::to_json(current_state_.timestamp_s) << "\n";
        json << "}";
        return json.str();
    }
    
    // Route request to appropriate handler
    std::string route_request(const std::string& method, const std::string& path) {
        // Only allow GET requests (read-only API)
        if (method != "GET") {
            return HTTPResponse::method_not_allowed();
        }
        
        try {
            if (path == "/health") {
                return HTTPResponse::ok(handle_health());
            } else if (path == "/api/battery") {
                return HTTPResponse::ok(handle_battery());
            } else if (path == "/api/torque") {
                return HTTPResponse::ok(handle_torque());
            } else if (path == "/api/regen") {
                return HTTPResponse::ok(handle_regen());
            } else if (path == "/api/limits") {
                return HTTPResponse::ok(handle_limits());
            } else if (path == "/api/diagnostics") {
                return HTTPResponse::ok(handle_diagnostics());
            } else if (path == "/api/energy_cycle") {
                return HTTPResponse::ok(handle_energy_cycle());
            } else {
                return HTTPResponse::not_found();
            }
        } catch (const std::exception& e) {
            return HTTPResponse::internal_error(e.what());
        }
    }
    
    // Main server loop (runs in dedicated thread)
    void server_loop() {
        while (running_.load()) {
            // Accept client connection
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_socket = accept(server_socket_, 
                                      (struct sockaddr*)&client_addr, 
                                      &client_len);
            
            if (client_socket < 0) {
                if (running_.load()) {
                    // Only report error if we're still supposed to be running
                    continue;
                } else {
                    break;
                }
            }
            
            // Set socket timeout to prevent hanging
            struct timeval timeout;
            timeout.tv_sec = 5;
            timeout.tv_usec = 0;
            setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            
            // Read HTTP request
            char buffer[4096];
            ssize_t bytes_read = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
            
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                std::string request(buffer);
                
                std::string method, path;
                if (parse_request(request, method, path)) {
                    std::string response = route_request(method, path);
                    send(client_socket, response.c_str(), response.length(), 0);
                } else {
                    std::string response = HTTPResponse::internal_error("Malformed request");
                    send(client_socket, response.c_str(), response.length(), 0);
                }
            }
            
            close(client_socket);
        }
    }
    
public:
    explicit HLVRestAPIServer(int port = 8080) 
        : port_(port), server_socket_(-1), running_(false) {
        // Initialize with default state
        current_state_.timestamp_s = 0.0;
    }
    
    ~HLVRestAPIServer() {
        stop();
    }
    
    // Start the API server
    bool start() {
        if (running_.load()) {
            return false; // Already running
        }
        
        // Create socket
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ < 0) {
            return false;
        }
        
        // Allow socket reuse
        int opt = 1;
        setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        // Bind to address
        struct sockaddr_in server_addr;
        std::memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY; // 0.0.0.0
        server_addr.sin_port = htons(port_);
        
        if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            close(server_socket_);
            return false;
        }
        
        // Listen for connections
        if (listen(server_socket_, 10) < 0) {
            close(server_socket_);
            return false;
        }
        
        // Start server thread
        running_.store(true);
        server_thread_ = std::thread(&HLVRestAPIServer::server_loop, this);
        
        return true;
    }
    
    // Stop the API server
    void stop() {
        if (!running_.load()) {
            return; // Not running
        }
        
        running_.store(false);
        
        // Close server socket to unblock accept()
        if (server_socket_ >= 0) {
            shutdown(server_socket_, SHUT_RDWR);
            close(server_socket_);
            server_socket_ = -1;
        }
        
        // Wait for server thread to finish
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }
    
    // Update battery state (thread-safe)
    void update_battery_state(const hlv::EnhancedState& enhanced_state,
                               const hlv_plugin::DiagnosticReport& diagnostics,
                               double timestamp_s) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.enhanced_state = enhanced_state;
        current_state_.diagnostics = diagnostics;
        current_state_.timestamp_s = timestamp_s;
    }
    
    // Update torque state (thread-safe)
    void update_torque_state(double max_drive_torque_nm,
                             double max_regen_torque_nm,
                             const std::string& limiting_factor,
                             double power_limit_kw) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.has_torque_data = true;
        current_state_.max_drive_torque_nm = max_drive_torque_nm;
        current_state_.max_regen_torque_nm = max_regen_torque_nm;
        current_state_.torque_limiting_factor = limiting_factor;
        current_state_.torque_power_limit_kw = power_limit_kw;
    }
    
    // Update regen state (thread-safe)
    void update_regen_state(double max_regen_torque_nm,
                            double regen_fraction,
                            const std::string& limiting_factor) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.has_regen_data = true;
        current_state_.regen_max_torque_nm = max_regen_torque_nm;
        current_state_.regen_fraction = regen_fraction;
        current_state_.regen_limiting_factor = limiting_factor;
    }
    
    // Update energy metrics (thread-safe)
    void update_energy_metrics(double energy_drive_kwh,
                               double energy_regen_kwh,
                               double energy_loss_kwh,
                               double delta_e_info_j,
                               double total_efficiency) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.has_energy_data = true;
        current_state_.energy_drive_kwh = energy_drive_kwh;
        current_state_.energy_regen_kwh = energy_regen_kwh;
        current_state_.energy_loss_kwh = energy_loss_kwh;
        current_state_.delta_e_info_j = delta_e_info_j;
        current_state_.total_efficiency = total_efficiency;
    }
    
    bool is_running() const {
        return running_.load();
    }
    
    int get_port() const {
        return port_;
    }
};

} // namespace api
} // namespace hlv

#endif // HLV_REST_API_SERVER_HPP
