# HLV EV Battery Enhancement - System Architecture

This document provides a highly detailed technical overview of the Helix-Light-Vortex (HLV) EV Battery Enhancement software structure, detailing module boundaries, interfaces, and the closed-loop energy cycle.

---

## 🏗️ Core Architecture Overview

The software integrates high-performance physical models (C++17) with robust scripting and distribution layers (Python/Shell) to create a highly reliable, deterministic, and safe automotive-adjacent control loop.

```
+------------------------------------------------------------+
|                       Top-Level CLI / UI                   |
|       (make install / diagnostics.py / hlv_enhancer)       |
+------------------------------------------------------------+
                              |
                              v
+------------------------------------------------------------+
|                pybind11 Python Wrapper                     |
|           (hlv_core/lib/hlv_enhancer_pybind.so)            |
+------------------------------------------------------------+
                              |
                              v (via C++ interfaces)
+------------------------------------------------------------+
|                    Compiled C++17 Core                     |
|          (libhlv_enhancer.so / hlv_enhancer binary)        |
+------------------------------------------------------------+
                              |
                              +---> hlv_bms_middleware_v2
                              +---> hlv_battery_core
                              +---> torque_enhancement
                              +---> hlv_regen_braking_manager_v1
```

---

## 🔁 The Closed-Loop Energy Cycle

The key differentiator of the HLV stack v4.1.0 is its closed-loop energy management cycle. Instead of treating battery state estimation, motor torque delivery, and braking energy recovery as isolated silos, they are integrated into a continuous physics-informed loop.

```
                  +--------------------------------+
                  |      Battery State (Ψ, Φ)      |
                  |  - Voltage, Current, SOH, SOC   |
                  |  - Entropy & Metric trace       |
                  +--------------------------------+
                    /                            \
                   /                              \
                  v                                v
+----------------------------+          +----------------------------+
|    HLV Torque Manager      |          |    HLV Regen Manager       |
|  - Progressive derating    |          |  - High-power recovery     |
|  - Thermal protections     |          |  - Adaptive blend ratio    |
|  - Limp mode enforcement   |          |  - Fail-safe mechanical cut|
+----------------------------+          +----------------------------+
                  \                                /
                   \                              /
                    v                            v
                  +--------------------------------+
                  |       Dynamic Vehicle          |
                  |       Energy Recovery          |
                  |  - Controlled charging updates |
                  |  - Closed-loop integration     |
                  +--------------------------------+
```

### 1. Battery State to Torque Delivery
* Sensor readings are updated at 100Hz. The `HLVBMSMiddleware` estimates the coupled thermodynamic state of the pack.
* When high internal stresses ($\partial_{\mu}\Phi$) or capacity degradation are detected, the `HLVTorqueManager` instantly scales back peak motor commands to prevent premature aging.

### 2. Kinetics to Regenerative Braking
* During deceleration, the driver's braking request is routed through the `HLVRegenBrakingManager`.
* Based on current SOC headroom, temperatures, and cell balances, the regen manager computes safe charge acceptance bounds.
* High-power energy recovery is permitted ONLY if cells are warm, balanced, and healthy.

### 3. Energy Recovery to Battery State
* The recovered energy is converted into charging current ($I_{\text{regen}}$) and fed back into the `HLVBMSMiddleware` update cycle.
* This updates the informational state (minimizing entropy growth) and updates physical State of Charge.
* **Outcome**: A safe, thermodynamics-compliant loop that reduces dependence on external grid charging while strictly preventing battery damage.

---

## 📂 Subsystem & Module Boundaries

### 1. High-Performance Core (`/hlv_core`)
Written in pure, standard C++17 with zero operating system dependencies:
* **`hlv_battery_core.hpp`**: Implements the dual-state $(\Psi, \Phi)$ system model. Physical attributes (voltage, current, temperature, state of charge) are coupled to informational attributes (entropy, history, degradation) using Marcel Krüger's effective metric formulation:
  $$g^{\text{eff}}_{\mu\nu} = g_{\mu\nu} + \lambda \partial_{\mu}\Phi \partial_{\nu}\Phi$$
* **`torque_enhancement.hpp`**: Translates battery health, entropy, and cell balances into intelligent real-time motor torque limits.
* **`hlv_regen_braking_manager_v1.hpp`**: Implements health-aware regenerative braking blending, allowing healthy cells to maximize energy recovery while protecting weak cells.

### 2. Python pybind11 Module Wrapper (`/hlv_core/src/hlv_pybind11_wrapper.cpp`)
* Implements high-performance C++ bindings that compile into `hlv_enhancer_pybind.so` using `pybind11`.
* Binds core classes (`HLVBMSMiddleware`, `HLVTorqueManager`), enums (`DriveMode`, `RegenMode`), and configurations (`TorqueConfig`, `MiddlewareConfig`), making them accessible to standard Python control scripts with zero parsing overhead.

### 3. Verification & Diagnostics Scripts (`/scripts`)
Modular, robust diagnostics tools used during installation, updates, or maintenance:
* **`diagnostics.py`**: The primary diagnostic script checking OS compliance, Python versions, interface existence, and physical battery boundaries (SOH >= 80%, SOC 20-90%, Temperature 0-45°C).
* **`update.py`**: Secure update manager that fetches updates, validates safety within staging, takes system backups, and deploys updates cleanly.
* **`rollback.sh`**: Handles backup generation and state restores during setup and software updates.

### 4. Configuration Layer (`/config`)
* **`vehicle_profiles.json`**: Supported EV models along with nominal voltages and firmware globs.
* **`safety_thresholds.json`**: Standard threshold definitions.
* **`install_policy.json`**: System-level installation rules and dependencies.
* **`vehicle_status.json`**: Mock state representing current simulated telemetries.
* **`update_source.json`**: Update source fallback URL and policy definitions.
