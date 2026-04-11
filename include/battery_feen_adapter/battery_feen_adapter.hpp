#ifndef BATTERY_FEEN_ADAPTER_HPP
#define BATTERY_FEEN_ADAPTER_HPP

// Do NOT include hlv_battery_core.hpp here to prevent circular dependency
// Forward declare the struct we need, or just pass individual values

#include <feen/ailee/confidence.h>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace hlv {

class BatteryFeenAdapter {
private:
    feen::ailee::PhononicConfidenceScorer scorer_;
    std::vector<double> voltage_history_;
    size_t max_history_size_;

public:
    BatteryFeenAdapter(size_t max_history_size = 10)
        : scorer_(feen::ailee::ConfidenceConfig{}),
          max_history_size_(max_history_size) {}

    double compute_battery_trust_from_feen(double current_voltage) {
        try {
            // Dummy peers logic (e.g., cell voltages could be used here if available,
            // but we only have a single pack voltage in HLVState)
            std::vector<double> peers = {current_voltage};

            feen::ailee::ConfidenceResult result = scorer_.evaluate(
                current_voltage,
                peers,
                voltage_history_
            );

            // Update history
            voltage_history_.push_back(current_voltage);
            if (voltage_history_.size() > max_history_size_) {
                voltage_history_.erase(voltage_history_.begin());
            }

            return result.score;
        } catch (...) {
            // Return safe default on any error
            return 1.0;
        }
    }

    void reset() {
        voltage_history_.clear();
    }
};

} // namespace hlv

#endif // BATTERY_FEEN_ADAPTER_HPP
