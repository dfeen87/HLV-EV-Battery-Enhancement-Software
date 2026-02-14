/*
 * ============================================================================
 * HLV REST API SERVER EXAMPLE
 * ============================================================================
 *
 * Demonstrates integration of HLV Battery Management REST API with the
 * HLV Enhancement system, torque management, and regenerative braking.
 *
 * This example:
 *   1. Initializes HLV battery middleware
 *   2. Starts REST API server on port 8080
 *   3. Simulates a drive cycle with varying load
 *   4. Updates API with real-time battery, torque, and regen state
 *   5. Runs until interrupted (Ctrl+C)
 *
 * USAGE:
 *   ./rest_api_server
 *   
 *   Then access endpoints:
 *     curl http://localhost:8080/health
 *     curl http://localhost:8080/api/battery
 *     curl http://localhost:8080/api/diagnostics
 *     etc.
 *
 * AUTHORS: Don Michael Feeney Jr. & Claude (Anthropic)
 * DATE: February 2026
 * LICENSE: MIT
 *
 * ============================================================================
 */

#include "hlv_rest_api_server.hpp"
#include "hlv_battery_enhancement.hpp"
#include "hlv_bms_middleware_v2.hpp"
#include "torque_enhancement.hpp"
#include "hlv_regen_braking_manager_v1.hpp"
#include "hlv_energy_telemetry.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

// Global flag for graceful shutdown
std::atomic<bool> shutdown_requested(false);

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\n[SERVER] Shutdown requested...\n";
        shutdown_requested.store(true);
    }
}

int main() {
    // Register signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    std::cout << "============================================================\n";
    std::cout << "HLV Battery Management REST API Server\n";
    std::cout << "Version: " << hlv::api::get_rest_api_version() << "\n";
    std::cout << "============================================================\n\n";
    
    try {
        // ====================================================================
        // INITIALIZE HLV COMPONENTS
        // ====================================================================
        
        std::cout << "[INIT] Initializing HLV Battery Middleware...\n";
        
        hlv_plugin::HLVBMSMiddleware middleware;
        middleware.init(75.0, 400.0); // 75 Ah, 400V nominal pack
        
        std::cout << "[INIT] Initializing Torque Manager...\n";
        
        hlv::drive::TorqueConfig torque_config;
        torque_config.drivetrain.has_rear_motor = true;
        torque_config.drivetrain.has_front_motor = false; // RWD
        torque_config.drivetrain.rear_motor.peak_torque_nm = 400.0;
        torque_config.drivetrain.rear_motor.continuous_torque_nm = 250.0;
        
        hlv::drive::HLVTorqueManager torque_manager(torque_config);
        
        std::cout << "[INIT] Initializing Regen Manager...\n";
        
        hlv::drive::RegenConfig regen_config;
        regen_config.peak_regen_torque_nm = 250.0;
        regen_config.max_regen_power_kw = 120.0;
        
        hlv::drive::HLVRegenBrakingManager regen_manager(regen_config);
        
        std::cout << "[INIT] Energy metrics tracking enabled...\n";
        
        // Simple energy tracking variables
        double total_energy_drive_kwh = 0.0;
        double total_energy_regen_kwh = 0.0;
        double total_energy_loss_kwh = 0.0;
        
        // ====================================================================
        // START REST API SERVER
        // ====================================================================
        
        std::cout << "[SERVER] Starting REST API server on port 8080...\n";
        
        hlv::api::HLVRestAPIServer api_server(8080);
        
        if (!api_server.start()) {
            std::cerr << "[ERROR] Failed to start REST API server!\n";
            std::cerr << "        Port 8080 may already be in use.\n";
            return 1;
        }
        
        std::cout << "[SERVER] REST API server started successfully!\n";
        std::cout << "[SERVER] Bind address: 0.0.0.0:8080\n\n";
        
        std::cout << "Available endpoints:\n";
        std::cout << "  GET http://localhost:8080/health\n";
        std::cout << "  GET http://localhost:8080/api/battery\n";
        std::cout << "  GET http://localhost:8080/api/torque\n";
        std::cout << "  GET http://localhost:8080/api/regen\n";
        std::cout << "  GET http://localhost:8080/api/limits\n";
        std::cout << "  GET http://localhost:8080/api/diagnostics\n";
        std::cout << "  GET http://localhost:8080/api/energy_cycle\n\n";
        
        std::cout << "Press Ctrl+C to stop the server.\n\n";
        std::cout << "============================================================\n\n";
        
        // ====================================================================
        // SIMULATION LOOP
        // ====================================================================
        
        double time_s = 0.0;
        double dt = 0.1; // 10 Hz update rate
        
        // Simulation parameters
        double voltage = 380.0;
        double soc = 0.85;
        double temperature = 25.0;
        double motor_rpm = 0.0;
        double vehicle_speed_kph = 0.0;
        
        int iteration = 0;
        
        while (!shutdown_requested.load()) {
            // ================================================================
            // SIMULATE DRIVE CYCLE
            // ================================================================
            
            // Simulate varying drive patterns (0-20 second cycle)
            double cycle_phase = std::fmod(time_s, 20.0);
            double current = 0.0;
            
            if (cycle_phase < 5.0) {
                // Acceleration phase
                current = 100.0 + 50.0 * (cycle_phase / 5.0);
                motor_rpm = 2000.0 + 2000.0 * (cycle_phase / 5.0);
                vehicle_speed_kph = 20.0 + 40.0 * (cycle_phase / 5.0);
            } else if (cycle_phase < 10.0) {
                // Cruising phase
                current = 50.0;
                motor_rpm = 4000.0;
                vehicle_speed_kph = 60.0;
            } else if (cycle_phase < 15.0) {
                // Regenerative braking phase
                double brake_phase = (cycle_phase - 10.0) / 5.0;
                current = -80.0 * brake_phase;
                motor_rpm = 4000.0 * (1.0 - brake_phase);
                vehicle_speed_kph = 60.0 * (1.0 - brake_phase);
            } else {
                // Coast / low speed
                current = 10.0;
                motor_rpm = 1000.0;
                vehicle_speed_kph = 10.0;
            }
            
            // Update voltage based on SoC (simplified model)
            voltage = 320.0 + 80.0 * soc;
            
            // Temperature increases slowly with usage
            temperature = 25.0 + 5.0 * std::sin(time_s * 0.05);
            
            // Update SoC based on current
            soc -= current * dt / (75.0 * 3600.0); // Simple coulomb counting
            soc = std::clamp(soc, 0.1, 0.95);
            
            // ================================================================
            // UPDATE HLV MIDDLEWARE
            // ================================================================
            
            auto enhanced_state = middleware.enhance_cycle(voltage, current, temperature, soc, dt);
            auto diagnostics = middleware.get_diagnostics();
            
            // ================================================================
            // UPDATE TORQUE MANAGER
            // ================================================================
            
            auto torque_result = torque_manager.compute_torque_limit(
                enhanced_state, 
                motor_rpm,
                dt,
                &diagnostics
            );
            
            // ================================================================
            // UPDATE REGEN MANAGER
            // ================================================================
            
            double brake_request = (cycle_phase >= 10.0 && cycle_phase < 15.0) ? 0.7 : 0.0;
            bool abs_active = false;
            bool wheel_slip = false;
            
            auto regen_result = regen_manager.compute_regen_limit(
                enhanced_state,
                &diagnostics,
                brake_request,
                vehicle_speed_kph,
                motor_rpm,
                abs_active,
                wheel_slip,
                dt
            );
            
            // ================================================================
            // UPDATE ENERGY TRACKING (simplified)
            // ================================================================
            
            // Calculate instantaneous power and energy
            double power_kw = voltage * current / 1000.0; // kW
            double energy_kwh = power_kw * dt / 3600.0; // kWh
            
            if (current > 0) {
                // Discharge (drive)
                total_energy_drive_kwh += energy_kwh;
                total_energy_loss_kwh += energy_kwh * 0.05; // 5% loss estimate
            } else if (current < 0) {
                // Charge (regen)
                total_energy_regen_kwh += -energy_kwh * 0.95; // 95% efficiency
                total_energy_loss_kwh += -energy_kwh * 0.05; // 5% loss
            }
            
            double total_efficiency = (total_energy_drive_kwh > 0) ? 
                (total_energy_regen_kwh / total_energy_drive_kwh) : 0.0;
            
            // ================================================================
            // UPDATE REST API STATE
            // ================================================================
            
            api_server.update_battery_state(enhanced_state, diagnostics, time_s);
            
            api_server.update_torque_state(
                torque_result.max_drive_torque_nm,
                torque_result.max_regen_torque_nm,
                torque_result.limiting_factor,
                torque_result.max_power_kw
            );
            
            api_server.update_regen_state(
                regen_result.max_regen_torque_nm,
                regen_result.regen_fraction,
                regen_result.limiting_factor
            );
            
            api_server.update_energy_metrics(
                total_energy_drive_kwh,
                total_energy_regen_kwh,
                total_energy_loss_kwh,
                enhanced_state.state.energy_phi, // ΔE_info approximation
                total_efficiency
            );
            
            // ================================================================
            // CONSOLE OUTPUT (periodic)
            // ================================================================
            
            if (iteration % 50 == 0) { // Every 5 seconds
                std::cout << std::fixed << std::setprecision(1);
                std::cout << "[" << std::setw(7) << time_s << "s] ";
                std::cout << "SoC:" << std::setw(5) << (soc * 100.0) << "% ";
                std::cout << "V:" << std::setw(5) << voltage << "V ";
                std::cout << "I:" << std::setw(6) << current << "A ";
                std::cout << "T:" << std::setw(4) << temperature << "°C ";
                std::cout << "Speed:" << std::setw(5) << vehicle_speed_kph << "kph ";
                std::cout << "Torque:" << std::setw(5) << torque_result.max_drive_torque_nm << "Nm ";
                std::cout << "Regen:" << std::setw(5) << regen_result.max_regen_torque_nm << "Nm";
                std::cout << "\n";
            }
            
            // ================================================================
            // TIMING
            // ================================================================
            
            time_s += dt;
            iteration++;
            
            // Sleep to maintain real-time rate (approximately)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // ====================================================================
        // SHUTDOWN
        // ====================================================================
        
        std::cout << "\n[SERVER] Stopping REST API server...\n";
        api_server.stop();
        
        std::cout << "[SERVER] Server stopped.\n";
        std::cout << "[SERVER] Total runtime: " << time_s << " seconds\n";
        std::cout << "[SERVER] Total iterations: " << iteration << "\n";
        
        std::cout << "\n============================================================\n";
        std::cout << "HLV REST API Server shutdown complete.\n";
        std::cout << "============================================================\n";
        
    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] Exception caught: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
