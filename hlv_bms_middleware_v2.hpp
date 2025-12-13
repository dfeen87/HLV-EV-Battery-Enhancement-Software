#ifndef HLV_BMS_MIDDLEWARE_V2_HPP
#define HLV_BMS_MIDDLEWARE_V2_HPP

#include "hlv_battery_enhancement.hpp"
#include "hlv_advanced_features.hpp"

#include <vector>
#include <array>
#include <string>
#include <memory>
#include <chrono>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <map>

namespace hlv_plugin {

/* ================= VERSION ================= */

constexpr int MIDDLEWARE_VERSION_MAJOR = 2;
constexpr int MIDDLEWARE_VERSION_MINOR = 0;
constexpr int MIDDLEWARE_VERSION_PATCH = 1;

inline std::string get_middleware_version() {
    return std::to_string(MIDDLEWARE_VERSION_MAJOR) + "." +
           std::to_string(MIDDLEWARE_VERSION_MINOR) + "." +
           std::to_string(MIDDLEWARE_VERSION_PATCH);
}

/* ================= DIAGNOSTICS ================= */

struct DiagnosticReport {
    double pack_soh_percent = 100.0;
    double pack_soc_percent = 100.0;
    double estimated_range_km = 0.0;
    double cycles_to_eol = 0.0;

    int total_cells = 1;
    int weak_cell_count = 0;
    std::vector<int> weak_cell_ids;

    double min_cell_voltage = 0.0;
    double max_cell_voltage = 0.0;
    double voltage_imbalance_mv = 0.0;

    double average_cell_temp_c = 25.0;
    double max_cell_temp_c = 25.0;
    double min_cell_temp_c = 25.0;

    double hlv_metric_trace = 2.0;
    double hlv_entropy = 0.0;
    double hlv_phi_magnitude = 0.0;
    double hlv_confidence = 1.0;

    bool weak_cell_warning = false;
    bool thermal_warning = false;
    bool voltage_warning = false;
    bool degradation_warning = false;
    bool balancing_required = false;
    bool safety_fault = false;

    double instantaneous_power_kw = 0.0;
    double average_efficiency = 0.95;
    double energy_throughput_kwh = 0.0;

    double time_since_init_s = 0.0;
    int update_count = 0;
};

/* ================= SAFETY LIMITS ================= */

struct SafetyLimits {
    double min_cell_voltage = 2.5;
    double max_cell_voltage = 4.3;
    double min_pack_voltage = 300.0;
    double max_pack_voltage = 450.0;

    double max_discharge_current = 600.0;
    double max_charge_current = 250.0;

    double min_operating_temp = -20.0;
    double max_operating_temp = 60.0;
    double max_cell_temp = 65.0;

    double min_soc = 0.05;
    double max_soc = 0.95;

    double max_voltage_imbalance_mv = 100.0;
    double min_health_percent = 70.0;
};

/* ================= CONFIG ================= */

struct MiddlewareConfig {
    enum class Mode { SIMPLE, MULTI_CELL_PACK, ADVANCED_ML };

    Mode mode = Mode::SIMPLE;
    hlv::advanced::ChemistryType chemistry = hlv::advanced::ChemistryType::NMC;

    double nominal_capacity_ah = 75.0;
    double nominal_voltage = 400.0;

    int series_cells = 96;

    hlv::HLVConfig hlv_config;

    bool enable_kalman_filter = false;
    bool enable_ml_hybrid = false;
    bool enable_cell_balancing = false;

    SafetyLimits safety_limits;
    bool enable_safety_monitoring = true;

    bool enable_logging = true;
    double logging_interval_s = 1.0;
};

/* ================= SAFETY MONITOR ================= */

class SafetyMonitor {
    SafetyLimits limits_;
    bool fault_active_ = false;
    std::vector<std::string> faults_;

public:
    explicit SafetyMonitor(const SafetyLimits& limits = {}) : limits_(limits) {}

    bool check(const hlv::HLVState& s, const DiagnosticReport& d) {
        faults_.clear();
        fault_active_ = false;

        if (s.current > limits_.max_discharge_current)
            faults_.push_back("Discharge current exceeded");

        if (s.current < -limits_.max_charge_current)
            faults_.push_back("Charge current exceeded");

        if (s.voltage < limits_.min_pack_voltage || s.voltage > limits_.max_pack_voltage)
            faults_.push_back("Pack voltage out of bounds");

        if (s.temperature > limits_.max_operating_temp)
            faults_.push_back("Temperature too high");

        if (d.max_cell_temp_c > limits_.max_cell_temp)
            faults_.push_back("Cell temperature critical");

        if (!faults_.empty()) fault_active_ = true;
        return !fault_active_;
    }

    bool has_fault() const { return fault_active_; }
    const std::vector<std::string>& faults() const { return faults_; }
};

/* ================= MIDDLEWARE ================= */

class HLVBMSMiddleware {
    MiddlewareConfig config_;

    std::unique_ptr<hlv::HLVEnhancement> hlv_core_;
    std::unique_ptr<hlv::advanced::MultiCellPack> pack_;
    std::unique_ptr<hlv::advanced::KalmanHLVFilter> kalman_;

    SafetyMonitor safety_;
    DiagnosticReport diag_;
    hlv::EnhancedState enhanced_;

    bool initialized_ = false;
    double time_s_ = 0.0;
    int updates_ = 0;

    double energy_in_kwh_ = 0.0;
    double energy_out_kwh_ = 0.0;

public:
    HLVBMSMiddleware() = default;

    void init(double capacity_ah, double voltage_v) {
        config_.nominal_capacity_ah = capacity_ah;
        config_.nominal_voltage = voltage_v;

        hlv_core_ = std::make_unique<hlv::HLVEnhancement>();
        hlv_core_->init(config_.hlv_config);

        safety_ = SafetyMonitor(config_.safety_limits);
        initialized_ = true;
    }

    void init_advanced(const MiddlewareConfig& cfg) {
        config_ = cfg;

        hlv_core_ = std::make_unique<hlv::HLVEnhancement>();
        hlv_core_->init(config_.hlv_config);

        if (cfg.mode != MiddlewareConfig::Mode::SIMPLE) {
            pack_ = std::make_unique<hlv::advanced::MultiCellPack>(
                cfg.series_cells,
                hlv::advanced::ChemistryLibrary().get_profile(cfg.chemistry));
        }

        if (cfg.enable_kalman_filter) {
            kalman_ = std::make_unique<hlv::advanced::KalmanHLVFilter>();
        }

        safety_ = SafetyMonitor(cfg.safety_limits);
        initialized_ = true;
    }

    hlv::EnhancedState enhance_cycle(double v, double i, double t, double soc, double dt) {
        if (!initialized_) throw std::runtime_error("Middleware not initialized");

        soc = std::clamp(soc, 0.0, 1.0);

        enhanced_ = hlv_core_->enhance(v, i, t, soc, dt);

        if (kalman_) {
            kalman_->predict(enhanced_.state, dt);
            kalman_->update(soc, i);
            auto f = kalman_->get_state();
            enhanced_.state.state_of_charge = std::clamp(f[0], 0.0, 1.0);
        }

        double power_kw = (v * i) / 1000.0;
        double e = power_kw * dt / 3600.0;

        if (i > 0) energy_out_kwh_ += e;
        else energy_in_kwh_ += std::abs(e);

        update_diagnostics(dt);
        return enhanced_;
    }

    const DiagnosticReport& diagnostics() const { return diag_; }
    const DiagnosticReport* diagnostics_ptr() const { return &diag_; }

// --------------------------------------------------------------------
    // Optional Telemetry Snapshot (Non-Control, Read-Only)
    // --------------------------------------------------------------------
    hlv::HLVEnergyTelemetry snapshot() const {
        hlv::HLVEnergyTelemetry t;

        t.soc_percent = diag_.pack_soc_percent;
        t.soh_percent = diag_.pack_soh_percent;

        t.pack_power_kw = diag_.instantaneous_power_kw;
        t.regen_power_kw = (diag_.instantaneous_power_kw < 0.0)
                             ? -diag_.instantaneous_power_kw
                             : 0.0;

        t.recovered_energy_kwh = total_energy_in_kwh_;
        t.hlv_metric_trace = diag_.hlv_metric_trace;
        t.hlv_entropy = diag_.hlv_entropy;
        t.hlv_confidence = diag_.hlv_confidence;

        t.limiting_factor = diag_.safety_fault ? "SAFETY" : "NONE";

        return t;
    }

private:
    void update_diagnostics(double dt) {
        auto& s = enhanced_.state;

        diag_.pack_soc_percent = s.state_of_charge * 100.0;
        diag_.pack_soh_percent = (1.0 - s.degradation) * 100.0;
        diag_.instantaneous_power_kw = (s.voltage * s.current) / 1000.0;
        diag_.hlv_metric_trace = s.g_eff.trace();
        diag_.hlv_entropy = s.entropy;
        diag_.hlv_phi_magnitude = s.phi_magnitude;
        diag_.hlv_confidence = enhanced_.hlv_confidence;

        if (config_.enable_safety_monitoring)
            diag_.safety_fault = !safety_.check(s, diag_);

        diag_.time_since_init_s += dt;
        diag_.update_count = ++updates_;
    }
};

} // namespace hlv_plugin

#endif


