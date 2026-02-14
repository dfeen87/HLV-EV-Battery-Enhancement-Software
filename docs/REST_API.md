# HLV Battery Management REST API

## Overview

The HLV REST API provides **read-only, observability-focused** HTTP/JSON endpoints for monitoring the state of the HLV Battery Management, Torque Enhancement, and Regenerative Braking systems.

### Key Features

- **Read-only endpoints** (GET only) - Safe for automotive use
- **JSON responses** - Easy integration with dashboards and monitoring tools
- **Thread-safe** - All shared data access is mutex-protected
- **Non-blocking** - Runs in dedicated thread, does not interfere with real-time control
- **Lightweight** - Pure C++17, POSIX sockets, no external dependencies
- **LAN-accessible** - Binds to `0.0.0.0:8080` for network-wide access

### Design Principles

1. **Observability Only** - No control surfaces; all endpoints are GET-only
2. **Automotive Safety** - Read-only API cannot affect vehicle control
3. **Real-time Compatible** - Non-blocking design preserves control loop timing
4. **Data Snapshots** - Thread-safe snapshots prevent data races
5. **Standards Compliant** - HTTP/1.1 with JSON responses

---

## Quick Start

### Starting the Server

```bash
# Build the project
mkdir build && cd build
cmake ..
make rest_api_server

# Run the server
./rest_api_server
```

The server will start on `http://0.0.0.0:8080` and begin serving API requests.

### Testing with curl

```bash
# Health check
curl http://localhost:8080/health

# Battery state
curl http://localhost:8080/api/battery

# Diagnostics
curl http://localhost:8080/api/diagnostics

# Pretty-print with jq
curl http://localhost:8080/api/battery | jq .
```

---

## API Endpoints

### GET /health

Server health check and version information.

**Response:**
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "service": "HLV Battery Management REST API",
  "timestamp": 123.456
}
```

**Fields:**
- `status` (string) - Server status ("healthy")
- `version` (string) - API version
- `service` (string) - Service name
- `timestamp` (number) - Current simulation/system time (seconds)

---

### GET /api/battery

Complete battery state including physical (Ψ), informational (Φ), and HLV metrics.

**Response:**
```json
{
  "physical_state": {
    "voltage_v": 380.5,
    "current_a": 125.3,
    "temperature_c": 27.2,
    "state_of_charge": 0.85,
    "energy_psi_j": 92340000.00
  },
  "informational_state": {
    "entropy": 0.234,
    "cycle_count": 145.67,
    "degradation": 0.052,
    "phi_magnitude": 0.241,
    "energy_phi_j": 3452.12
  },
  "hlv_metrics": {
    "lambda": 0.000001000,
    "metric_trace": 2.000345,
    "energy_metric_j": 23.45,
    "energy_total_j": 92343475.57
  },
  "health": {
    "remaining_capacity_percent": 94.8,
    "cycles_to_80_percent": 1854.3,
    "confidence": 0.92,
    "warning_triggered": false
  },
  "timestamp_s": 123.456
}
```

**Physical State (Ψ):**
- `voltage_v` - Pack voltage (Volts)
- `current_a` - Pack current (Amperes, positive = discharge, negative = charge)
- `temperature_c` - Average pack temperature (°C)
- `state_of_charge` - State of charge (0.0 - 1.0)
- `energy_psi_j` - Physical energy stored (Joules)

**Informational State (Φ):**
- `entropy` - Normalized information entropy (0.0 - 1.0)
- `cycle_count` - Equivalent full discharge/charge cycles
- `degradation` - Degradation level (0.0 = new, 1.0 = end-of-life)
- `phi_magnitude` - Magnitude of informational field |Φ|
- `energy_phi_j` - Informational energy (Landauer principle, Joules)

**HLV Metrics:**
- `lambda` - HLV coupling strength λ
- `metric_trace` - Trace of effective metric tensor g^eff_μν
- `energy_metric_j` - Energy in metric modulation (Joules)
- `energy_total_j` - Total energy (should be conserved)

**Health:**
- `remaining_capacity_percent` - Predicted remaining capacity (%)
- `cycles_to_80_percent` - Cycles until 80% capacity threshold
- `confidence` - HLV prediction confidence (0.0 - 1.0)
- `warning_triggered` - Early degradation warning flag

---

### GET /api/torque

Torque limits and limiting factors from the torque management system.

**Response:**
```json
{
  "available": true,
  "max_drive_torque_nm": 380.5,
  "max_regen_torque_nm": 220.0,
  "power_limit_kw": 245.3,
  "limiting_factor": "BATTERY_CURRENT",
  "timestamp_s": 123.456
}
```

**Fields:**
- `available` (bool) - Whether torque data is available
- `max_drive_torque_nm` - Maximum drive torque (Newton-meters)
- `max_regen_torque_nm` - Maximum regenerative torque (Newton-meters)
- `power_limit_kw` - Battery power limit (kilowatts)
- `limiting_factor` - What's limiting torque (e.g., BATTERY_CURRENT, THERMAL, VOLTAGE, etc.)
- `timestamp_s` - Snapshot timestamp (seconds)

**Limiting Factors:**
- `BATTERY_CURRENT` - Battery discharge current limit
- `BATTERY_VOLTAGE` - Battery voltage limit (low or high)
- `BATTERY_SOC` - State of charge limit (low or high)
- `THERMAL` - Thermal derating active
- `MOTOR_THERMAL` - Motor temperature limit
- `INVERTER_THERMAL` - Inverter temperature limit
- `NONE` - No active limits

---

### GET /api/regen

Regenerative braking limits and charge acceptance constraints.

**Response:**
```json
{
  "available": true,
  "max_regen_torque_nm": 195.2,
  "regen_fraction": 0.78,
  "limiting_factor": "SOC_HEADROOM",
  "charge_acceptance": {
    "soc": 0.85,
    "temperature_c": 27.2,
    "voltage_v": 380.5
  },
  "timestamp_s": 123.456
}
```

**Fields:**
- `available` (bool) - Whether regen data is available
- `max_regen_torque_nm` - Maximum safe regen torque (Newton-meters)
- `regen_fraction` - Recommended regen fraction for brake blending (0.0 - 1.0)
- `limiting_factor` - What's limiting regen capability
- `charge_acceptance` - Current battery charge acceptance factors
  - `soc` - Current state of charge
  - `temperature_c` - Battery temperature
  - `voltage_v` - Pack voltage
- `timestamp_s` - Snapshot timestamp (seconds)

**Regen Limiting Factors:**
- `SOC_HEADROOM` - High SOC, limited charge acceptance
- `VOLTAGE_HEADROOM` - Pack voltage near maximum
- `TEMPERATURE_COLD` - Low temperature reduces charge acceptance
- `TEMPERATURE_HOT` - High temperature safety limit
- `CELL_IMBALANCE` - Cell voltage imbalance detected
- `HLV_STRESS` - HLV metrics indicate battery stress
- `ABS_ACTIVE` - ABS/ESC intervention, regen reduced
- `NONE` - No active regen limits

---

### GET /api/limits

All system limits in one consolidated view.

**Response:**
```json
{
  "battery": {
    "min_soc": 0.05,
    "max_soc": 0.95,
    "min_voltage_v": 307.2,
    "max_voltage_v": 412.8,
    "min_temp_c": -20.0,
    "max_temp_c": 60.0
  },
  "power": {
    "max_discharge_kw": 245.3,
    "max_charge_kw": 120.0
  },
  "torque": {
    "max_drive_nm": 380.5,
    "max_regen_nm": 220.0
  },
  "timestamp_s": 123.456
}
```

**Battery Limits:**
- `min_soc`, `max_soc` - Operating SOC range
- `min_voltage_v`, `max_voltage_v` - Pack voltage limits
- `min_temp_c`, `max_temp_c` - Operating temperature range

**Power Limits:**
- `max_discharge_kw` - Maximum discharge power
- `max_charge_kw` - Maximum charge (regen) power

**Torque Limits:**
- `max_drive_nm` - Maximum drive torque
- `max_regen_nm` - Maximum regen torque

---

### GET /api/diagnostics

Comprehensive pack diagnostics including cell-level data, thermal spread, and warnings.

**Response:**
```json
{
  "pack": {
    "soh_percent": 95.2,
    "health_percent": 94.8,
    "total_cells": 96,
    "weak_cell_count": 2
  },
  "voltage": {
    "min_cell_v": 3.82,
    "max_cell_v": 3.88,
    "imbalance_mv": 60.0,
    "voltage_warning": false
  },
  "thermal": {
    "average_temp_c": 27.2,
    "max_temp_c": 29.5,
    "min_temp_c": 25.8,
    "thermal_spread_c": 3.7,
    "thermal_warning": false
  },
  "hlv_metrics": {
    "metric_trace": 2.000345,
    "entropy": 0.234,
    "phi_magnitude": 0.241,
    "confidence": 0.92
  },
  "warnings": {
    "weak_cell": false,
    "thermal": false,
    "voltage": false,
    "degradation": false,
    "balancing_required": false,
    "safety_fault": false
  },
  "timestamp_s": 123.456
}
```

**Pack Diagnostics:**
- `soh_percent` - State of Health (%)
- `health_percent` - Overall pack health (%)
- `total_cells` - Total cells in pack
- `weak_cell_count` - Number of weak cells detected

**Voltage Diagnostics:**
- `min_cell_v`, `max_cell_v` - Cell voltage range
- `imbalance_mv` - Voltage imbalance (millivolts)
- `voltage_warning` - Voltage-related warning active

**Thermal Diagnostics:**
- `average_temp_c` - Average cell temperature
- `max_temp_c`, `min_temp_c` - Temperature range
- `thermal_spread_c` - Temperature spread across pack
- `thermal_warning` - Thermal warning active

**HLV Metrics:**
- `metric_trace` - HLV metric tensor trace
- `entropy` - Information entropy level
- `phi_magnitude` - Informational field magnitude
- `confidence` - HLV prediction confidence

**Warnings:**
- `weak_cell` - Weak cell detected
- `thermal` - Thermal issue detected
- `voltage` - Voltage issue detected
- `degradation` - Accelerated degradation detected
- `balancing_required` - Cell balancing needed
- `safety_fault` - Critical safety fault active

---

### GET /api/energy_cycle

Energy cycle metrics including drive, regen, losses, and informational energy.

**Response:**
```json
{
  "available": true,
  "energy_drive_kwh": 45.23,
  "energy_regen_kwh": 8.76,
  "energy_loss_kwh": 4.12,
  "delta_e_info_j": 3452.12,
  "total_efficiency": 0.912,
  "cycle_metrics": {
    "cycle_count": 145.67,
    "charge_throughput_ah": 21850.5,
    "capacity_fade": 0.052
  },
  "timestamp_s": 123.456
}
```

**Energy Metrics:**
- `available` (bool) - Whether energy data is available
- `energy_drive_kwh` - Total drive energy consumed (kWh)
- `energy_regen_kwh` - Total regenerative energy recovered (kWh)
- `energy_loss_kwh` - Total energy losses (kWh)
- `delta_e_info_j` - Change in informational energy ΔE_info (Joules)
- `total_efficiency` - Round-trip energy efficiency (0.0 - 1.0)

**Cycle Metrics:**
- `cycle_count` - Equivalent full cycles
- `charge_throughput_ah` - Total charge throughput (Amp-hours)
- `capacity_fade` - Measured capacity fade (0.0 - 1.0)

---

## Integration Examples

### Python Example

```python
import requests
import json

# Query battery state
response = requests.get('http://localhost:8080/api/battery')
data = response.json()

print(f"SOC: {data['physical_state']['state_of_charge'] * 100:.1f}%")
print(f"Voltage: {data['physical_state']['voltage_v']:.1f}V")
print(f"Current: {data['physical_state']['current_a']:.1f}A")
print(f"Health: {data['health']['remaining_capacity_percent']:.1f}%")
```

### JavaScript Example

```javascript
// Fetch battery diagnostics
fetch('http://localhost:8080/api/diagnostics')
  .then(response => response.json())
  .then(data => {
    console.log(`Pack Health: ${data.pack.health_percent}%`);
    console.log(`Weak Cells: ${data.pack.weak_cell_count}`);
    console.log(`Temp Spread: ${data.thermal.thermal_spread_c}°C`);
  });
```

### Shell Script Example

```bash
#!/bin/bash

# Monitor battery state every 5 seconds
while true; do
  clear
  echo "HLV Battery Monitor"
  echo "==================="
  curl -s http://localhost:8080/api/battery | jq '{
    soc: .physical_state.state_of_charge,
    voltage: .physical_state.voltage_v,
    current: .physical_state.current_a,
    temp: .physical_state.temperature_c,
    health: .health.remaining_capacity_percent
  }'
  sleep 5
done
```

### C++ Example

See `examples/rest_api_client.cpp` for a complete C++ client implementation.

---

## Security Considerations

### Read-Only Design

- **All endpoints are GET-only** - No POST/PUT/DELETE/PATCH allowed
- **No control surfaces** - API cannot modify vehicle state
- **Safe for automotive use** - Cannot interfere with real-time control

### Network Security

- **LAN-only deployment recommended** - Bind to trusted network segment
- **No authentication** - API assumes trusted network environment
- **Add reverse proxy if needed** - Use nginx/Apache for authentication/TLS
- **Firewall rules** - Restrict access to port 8080 as needed

### Thread Safety

- **Mutex-protected state** - All shared data access is synchronized
- **Non-blocking design** - API server runs in dedicated thread
- **Snapshot-based** - Responses contain consistent data snapshots

---

## Performance

### Typical Response Times

- Health check: < 1ms
- Simple endpoints (battery, torque, regen): 1-2ms
- Complex endpoints (diagnostics, energy): 2-5ms

### Throughput

- **Concurrent requests**: Handled sequentially (one at a time)
- **Request rate**: ~100-500 requests/second depending on endpoint
- **Thread overhead**: < 0.1% CPU on typical embedded platform

### Memory Footprint

- Server code: ~30KB
- Per-connection overhead: ~8KB
- Total typical usage: ~50KB

---

## Troubleshooting

### Server Won't Start

**Problem**: "Failed to start REST API server"

**Solutions**:
- Check if port 8080 is already in use: `netstat -tuln | grep 8080`
- Kill existing process: `sudo kill $(lsof -t -i:8080)`
- Change port in code and rebuild

### Connection Refused

**Problem**: "Connection refused" when querying API

**Solutions**:
- Verify server is running: `ps aux | grep rest_api_server`
- Check firewall rules: `sudo iptables -L | grep 8080`
- Ensure binding to 0.0.0.0, not 127.0.0.1 only

### Empty/Invalid Responses

**Problem**: JSON responses are empty or malformed

**Solutions**:
- Check that HLV middleware is properly initialized
- Verify update_*_state() calls are being made in main loop
- Inspect server console output for errors

---

## API Version History

### Version 1.0.0 (February 2026)

Initial release with:
- 7 endpoints (health, battery, torque, regen, limits, diagnostics, energy_cycle)
- Thread-safe state management
- JSON responses
- POSIX socket-based HTTP server
- Read-only design for automotive safety

---

## References

- HLV Theory: Krüger, M. (2025). "Mathematical Formulation of U2→U1 Coupling"
- Repository: https://github.com/dfeen87/HLV-EV-Battery-Enhancement-Software
- Examples: See `examples/rest_api_server.cpp` and `examples/rest_api_client.cpp`

---

## Support

For questions, issues, or contributions:
- GitHub Issues: https://github.com/dfeen87/HLV-EV-Battery-Enhancement-Software/issues
- Documentation: https://github.com/dfeen87/HLV-EV-Battery-Enhancement-Software/docs

---

**Last Updated**: February 2026  
**API Version**: 1.0.0
