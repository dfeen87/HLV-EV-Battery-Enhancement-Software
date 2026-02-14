#!/bin/bash
# Simple example script demonstrating REST API usage with curl and jq

API_BASE="http://localhost:8080"

echo "========================================"
echo "HLV Battery Management REST API Demo"
echo "========================================"
echo ""

# Check server health
echo "1. Server Health:"
curl -s "$API_BASE/health" | jq .
echo ""

# Get battery state
echo "2. Battery State (SOC, Voltage, Temperature):"
curl -s "$API_BASE/api/battery" | jq '{
  soc_percent: (.physical_state.state_of_charge * 100),
  voltage_v: .physical_state.voltage_v,
  current_a: .physical_state.current_a,
  temperature_c: .physical_state.temperature_c,
  health_percent: .health.remaining_capacity_percent
}'
echo ""

# Get diagnostics
echo "3. Pack Diagnostics:"
curl -s "$API_BASE/api/diagnostics" | jq '{
  soh: .pack.soh_percent,
  total_cells: .pack.total_cells,
  weak_cells: .pack.weak_cell_count,
  temp_spread: .thermal.thermal_spread_c,
  voltage_imbalance_mv: .voltage.imbalance_mv,
  warnings: .warnings
}'
echo ""

# Get torque limits
echo "4. Torque Limits:"
curl -s "$API_BASE/api/torque" | jq '{
  max_drive_nm: .max_drive_torque_nm,
  max_regen_nm: .max_regen_torque_nm,
  power_limit_kw: .power_limit_kw,
  limiting_factor: .limiting_factor
}'
echo ""

# Get energy metrics
echo "5. Energy Cycle Metrics:"
curl -s "$API_BASE/api/energy_cycle" | jq '{
  drive_kwh: .energy_drive_kwh,
  regen_kwh: .energy_regen_kwh,
  efficiency: .total_efficiency,
  cycle_count: .cycle_metrics.cycle_count
}'
echo ""

echo "========================================"
echo "Demo complete!"
echo "========================================"
