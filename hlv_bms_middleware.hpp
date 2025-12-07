/*
 * ============================================================================
 * HLV BMS MIDDLEWARE PLUGIN
 * ============================================================================
 *
 * This file demonstrates how to create a middleware layer that integrates
 * the HLV Battery Enhancement Library with an existing EV Battery Management
 * System (BMS).
 *
 * The middleware acts as a "plug-in" that:
 *   1. Receives raw sensor readings from the existing BMS.
 *   2. Calls HLV to compute enhanced battery state, health predictions,
 *      and optimal charging recommendations.
 *   3. Returns the enhanced data back to the BMS for improved decision-making.
 *
 * Advantages:
 *   - Minimal intrusion into proprietary BMS code
 *   - Predictive degradation monitoring
 *   - Optimal charging and entropy-aware battery management
 *   - Real-time compatible, lightweight
 *
 * ============================================================================
 */

#ifndef HLV_BMS_MIDDLEWARE_HPP
#define HLV_BMS_MIDDLEWARE_HPP

#include "hlv_battery_enhancement.hpp"
#include <stdexcept>

namespace hlv_plugin {

/*
 * HLVBMSMiddleware
 *
 * Acts as the interface between your existing BMS and the HLV enhancement layer.
 * Wraps HLVEnhancement with a clean API.
 */
class HLVBMSMiddleware {
private:
    hlv::HLVEnhancement hlv_;
    bool initialized_;

public:
    // Constructor
    HLVBMSMiddleware() : initialized_(false) {}

    // Initialize the HLV plugin with battery-specific parameters
    void init(double nominal_capacity_ah, double nominal_voltage,
              double max_temperature = 60.0) {
        hlv::HLVConfig config;
        config.nominal_capacity_ah = nominal_capacity_ah;
        config.nominal_voltage = nominal_voltage;
        config.max_temperature = max_temperature;

        hlv_.init(config);
        initialized_ = true;
    }

    // Enhance a single BMS update cycle
    // Inputs: sensor readings from the BMS
    // Outputs: enhanced battery state and recommendations
    hlv::EnhancedState enhance_cycle(double voltage, double current,
                                     double temperature, double soc,
                                     double dt = 0.1) {
        if (!initialized_) {
            throw std::runtime_error("HLVBMSMiddleware not initialized.");
        }

        // Call HLV library to compute enhanced state
        return hlv_.enhance(voltage, current, temperature, soc, dt);
    }

    // Optional helpers for long-term health forecasts
    hlv::HealthPrediction get_health_forecast(double cycles_ahead) {
        return hlv_.get_health_forecast(cycles_ahead);
    }

    hlv::OptimalChargingProfile get_optimal_charging() {
        return hlv_.get_optimal_charging();
    }
};

} // namespace hlv_plugin

#endif // HLV_BMS_MIDDLEWARE_HPP

/*
 * ============================================================================
 * EXAMPLE USAGE:
 *
 * #include "hlv_bms_middleware.hpp"
 *
 * hlv_plugin::HLVBMSMiddleware middleware;
 * middleware.init(75.0, 400.0);
 *
 * // Inside your existing BMS update loop:
 * auto enhanced = middleware.enhance_cycle(voltage, current, temp, soc, dt);
 *
 * if (enhanced.degradation_warning) {
 *     trigger_maintenance_alert();
 * }
 *
 * double optimal_current = enhanced.charging.recommended_current_limit;
 * double optimal_voltage = enhanced.charging.recommended_voltage_limit;
 *
 * ============================================================================
 */
