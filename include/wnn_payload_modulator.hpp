#ifndef WNN_PAYLOAD_MODULATOR_HPP
#define WNN_PAYLOAD_MODULATOR_HPP

#include <chrono>
#include <cmath>
#include <numbers>
#include <type_traits>

#include "hlv_wnn_telemetry_bridge.hpp"

namespace hlv_wnn {

/**
 * @brief Represents a state value and its Kahan compensation term.
 * Maintains strict long double precision to maintain mathematical
 * determinism with WNN's Kahan summation algorithm.
 */
struct KahanState {
    long double value = 0.0L;
    long double c = 0.0L; // compensation
};

/**
 * @brief Adds a term to a Kahan state to prevent floating point drift.
 */
inline void kahan_add(KahanState& state, long double term) {
    long double y = term - state.c;
    long double t = state.value + y;
    state.c = (t - state.value) - y;
    state.value = t;
}

/**
 * @brief Modulates WNN Duffing Oscillator wave parameters based on HLV telemetry.
 */
class WNNPayloadModulator {
public:
    explicit WNNPayloadModulator(HLVWNNBridge& bridge) : bridge_(bridge) {}

    /**
     * @brief Polls the telemetry from the bridge's lock-free ring buffer.
     * Non-blocking, zero-latency hot path.
     * @return True if a new payload was ingested, false otherwise.
     */
    bool poll_telemetry() {
        return bridge_.consume_payload(latest_payload_);
    }

    /**
     * @brief Computes Marcel Krüger's Spiral-Time phase salt.
     * This acts as a physical nonce to prevent replay attacks.
     *
     * Computed deterministically based on simulation time `t` to preserve
     * RK4 phase coherence, preventing chaotic stochastic noise mid-step.
     *
     * Formula: salt = fmod(t * golden_ratio, 2*pi)
     */
    long double calculate_spiral_time_salt(long double t) const {
        // Golden ratio conjugate
        constexpr long double phi = 1.6180339887498948482L;

        // Note: Simulation time `t` is used instead of wall-clock time
        // to maintain absolute determinism and numerical stability within
        // the RK4 integration loop, while avoiding catastrophic cancellation.
        return std::fmod(t * phi, 2.0L * std::numbers::pi_v<long double>);
    }

    /**
     * @brief Calculates the modulated forcing function for the Duffing oscillator.
     * γ cos(ωt + phase_shift)
     *
     * γ is modulated by voltage and SoC.
     * phase_shift is modulated by entropy and metric trace, plus the spiral time salt.
     */
    long double calculate_forcing(long double t, long double base_gamma, long double base_omega) const {
        // Amplitude modulation
        long double gamma = base_gamma;
        if (latest_payload_.voltage > 0.0L) {
            gamma += latest_payload_.voltage * 0.001L; // Slight modulation
            gamma *= (0.5L + 0.5L * latest_payload_.state_of_charge);
        }

        // Phase shift modulation
        long double phase_shift = 0.0L;
        phase_shift += latest_payload_.entropy_accumulation * 0.1L;
        phase_shift += latest_payload_.metric_trace * 0.01L;

        // Spiral-Time Phase Salting
        long double salt = calculate_spiral_time_salt(t);
        phase_shift += salt;

        return gamma * std::cos(base_omega * t + phase_shift);
    }

    const WNNThermodynamicPayload& get_latest_payload() const {
        return latest_payload_;
    }

private:
    HLVWNNBridge& bridge_;
    WNNThermodynamicPayload latest_payload_{};
};

/**
 * @brief Deterministic transduction engine using 4th-order Runge-Kutta.
 * Integrates the Duffing oscillator equation: ẍ + δẋ - αx + βx³ = γ cos(ωt + phase_shift)
 */
class WNNTransductionEngine {
public:
    struct DuffingParams {
        long double delta = 0.1L;
        long double alpha = 1.0L;
        long double beta = 1.0L;
        long double base_gamma = 0.5L;
        long double base_omega = 1.2L;
    };

    WNNTransductionEngine(WNNPayloadModulator& modulator, const DuffingParams& params)
        : modulator_(modulator), params_(params) {}

    /**
     * @brief Integrates the state forward by dt using Kahan-summed RK4.
     */
    void step(long double dt) {
        // Poll for fresh telemetry without blocking
        modulator_.poll_telemetry();

        long double t = time_.value;
        long double x = state_x_.value;
        long double v = state_v_.value; // dx/dt

        // RK4 Step 1
        long double k1_x = v;
        long double k1_v = acceleration(t, x, v);

        // RK4 Step 2
        long double k2_x = v + 0.5L * dt * k1_v;
        long double k2_v = acceleration(t + 0.5L * dt, x + 0.5L * dt * k1_x, k2_x);

        // RK4 Step 3
        long double k3_x = v + 0.5L * dt * k2_v;
        long double k3_v = acceleration(t + 0.5L * dt, x + 0.5L * dt * k2_x, k3_x);

        // RK4 Step 4
        long double k4_x = v + dt * k3_v;
        long double k4_v = acceleration(t + dt, x + dt * k3_x, k4_x);

        // Accumulate with Kahan summation to prevent floating point drift
        long double dx = (dt / 6.0L) * (k1_x + 2.0L * k2_x + 2.0L * k3_x + k4_x);
        long double dv = (dt / 6.0L) * (k1_v + 2.0L * k2_v + 2.0L * k3_v + k4_v);

        kahan_add(state_x_, dx);
        kahan_add(state_v_, dv);
        kahan_add(time_, dt);
    }

    long double get_x() const { return state_x_.value; }
    long double get_v() const { return state_v_.value; }
    long double get_time() const { return time_.value; }

private:
    /**
     * @brief Calculates acceleration (ẍ) based on the Duffing oscillator equation.
     * ẍ = -δẋ + αx - βx³ + γ cos(ωt + phase_shift)
     */
    long double acceleration(long double t, long double x, long double v) const {
        long double forcing = modulator_.calculate_forcing(t, params_.base_gamma, params_.base_omega);
        return -params_.delta * v + params_.alpha * x - params_.beta * (x * x * x) + forcing;
    }

    WNNPayloadModulator& modulator_;
    DuffingParams params_;

    KahanState state_x_; // Position
    KahanState state_v_; // Velocity
    KahanState time_;    // Simulation time
};

} // namespace hlv_wnn

#endif // WNN_PAYLOAD_MODULATOR_HPP
