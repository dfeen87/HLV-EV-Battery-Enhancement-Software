/*
 * ============================================================================
 * HLV REST API CLIENT EXAMPLE
 * ============================================================================
 *
 * Simple C++ client demonstrating how to query the HLV REST API.
 * Shows how to fetch and parse JSON responses from the API endpoints.
 *
 * USAGE:
 *   # First, start the server in another terminal:
 *   ./rest_api_server
 *   
 *   # Then run this client:
 *   ./rest_api_client
 *
 * ENDPOINTS TESTED:
 *   - GET /health
 *   - GET /api/battery
 *   - GET /api/torque
 *   - GET /api/regen
 *   - GET /api/diagnostics
 *   - GET /api/energy_cycle
 *
 * NOTE: This is a simple demonstration client. For production use,
 *       consider using a proper HTTP library like libcurl or a JSON
 *       parsing library like nlohmann/json.
 *
 * AUTHORS: Don Michael Feeney Jr. & Claude (Anthropic)
 * DATE: February 2026
 * LICENSE: MIT
 *
 * ============================================================================
 */

#include <iostream>
#include <string>
#include <sstream>
#include <cstring>
#include <vector>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// Simple HTTP GET request helper
std::string http_get(const std::string& host, int port, const std::string& path) {
    // Create socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        return "[ERROR] Failed to create socket";
    }
    
    // Set socket timeout
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
    // Connect to server
    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
        close(sock);
        return "[ERROR] Invalid address";
    }
    
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        close(sock);
        return "[ERROR] Connection failed";
    }
    
    // Build HTTP request
    std::ostringstream request;
    request << "GET " << path << " HTTP/1.1\r\n";
    request << "Host: " << host << "\r\n";
    request << "Connection: close\r\n";
    request << "\r\n";
    
    std::string request_str = request.str();
    
    // Send request
    if (send(sock, request_str.c_str(), request_str.length(), 0) < 0) {
        close(sock);
        return "[ERROR] Failed to send request";
    }
    
    // Read response
    std::string response;
    char buffer[4096];
    ssize_t bytes_read;
    
    while ((bytes_read = recv(sock, buffer, sizeof(buffer) - 1, 0)) > 0) {
        buffer[bytes_read] = '\0';
        response += buffer;
    }
    
    close(sock);
    
    // Extract body from response (skip HTTP headers)
    size_t body_start = response.find("\r\n\r\n");
    if (body_start != std::string::npos) {
        return response.substr(body_start + 4);
    }
    
    return response;
}

// Pretty print JSON response (simple indentation)
void print_response(const std::string& endpoint, const std::string& json) {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "Endpoint: " << endpoint << "\n";
    std::cout << std::string(70, '=') << "\n";
    
    // Simple pretty-printing (just add newlines after commas for readability)
    std::string formatted = json;
    for (size_t i = 0; i < formatted.length(); ++i) {
        if (formatted[i] == '{' || formatted[i] == '[') {
            std::cout << formatted[i] << "\n  ";
        } else if (formatted[i] == '}' || formatted[i] == ']') {
            std::cout << "\n" << formatted[i];
        } else if (formatted[i] == ',') {
            std::cout << formatted[i] << "\n  ";
        } else {
            std::cout << formatted[i];
        }
    }
    std::cout << "\n";
}

int main() {
    const std::string host = "127.0.0.1";
    const int port = 8080;
    
    std::cout << "============================================================\n";
    std::cout << "HLV REST API Client Example\n";
    std::cout << "============================================================\n";
    std::cout << "Connecting to: " << host << ":" << port << "\n";
    std::cout << "============================================================\n";
    
    // Test each endpoint
    std::vector<std::string> endpoints = {
        "/health",
        "/api/battery",
        "/api/torque",
        "/api/regen",
        "/api/limits",
        "/api/diagnostics",
        "/api/energy_cycle"
    };
    
    for (const auto& endpoint : endpoints) {
        std::string response = http_get(host, port, endpoint);
        print_response(endpoint, response);
        
        // Small delay between requests
        usleep(100000); // 100ms
    }
    
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "All endpoints tested successfully!\n";
    std::cout << std::string(70, '=') << "\n\n";
    
    std::cout << "TIP: You can also use curl to query the API:\n";
    std::cout << "  curl http://localhost:8080/health\n";
    std::cout << "  curl http://localhost:8080/api/battery | jq .\n";
    std::cout << "  curl http://localhost:8080/api/diagnostics | jq .\n\n";
    
    return 0;
}
