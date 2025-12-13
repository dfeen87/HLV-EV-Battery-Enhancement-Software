#pragma once

namespace hlv {

/*
 * ============================================================================
 * HLV Energy Telemetry (Optional)
 * ============================================================================
 *
 * PURPOSE:
 *   Provide a minimal, OEM-friendly snapshot of HLV energy state for
 *   dashboards, logging, CAN mapping, or cloud pipelines.
 *
 * DESIGN PRINCIPLES:
 *   - No UI assumptions
 *   - No timing assumptions
 *   - No ownership of visualization
 *   - Read-only, snapshot-based
 *
 * This struct is intentionally small and stable.
 * ============================================================================
 */

struct HLVEnergyTelemetry {
    // Battery state
    double soc_percent = 0.0;
    double soh_percent = 0.0;

    // Power flow
    double pack_power_kw = 0.0;        // signed: +discharge / -charge
    double regen_power_kw = 0.0;       // positive when recovering
    double recovered_energy_kwh = 0.0;

    // HLV metrics
    double hlv_metric_trace = 0.0;
    double hlv_entropy = 0.0;
    double hlv_confidence = 0.0;

    // Control context
    const char* limiting_factor = "NONE"; // e.g. SOC, THERMAL, CELL, ABS
};

} // namespace hlv
