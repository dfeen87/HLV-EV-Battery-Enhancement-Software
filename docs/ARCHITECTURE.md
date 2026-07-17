# HLV EV Battery Enhancement - System Architecture

This document provides a technical overview of the Helix-Light-Vortex (HLV) EV Battery Enhancement software structure, detailing the interaction of its subsystems.

---

## 🏗️ Core Architecture Overview

The software integrates high-performance physical models (C++17) with robust scripting and distribution layers (Python/Shell) to create a highly reliable, deterministic, and safe automotive-adjacent control loop.

```
+------------------------------------------------------------+
|                       Top-Level CLI / UI                   |
|           (make install / diagnostics.py / hlv_enhancer)    |
+------------------------------------------------------------+
                              |
                              v
+------------------------------------------------------------+
|                   Python Wrapper Layer                     |
|                 (hlv_core/python/hlv_enhancer.py)          |
+------------------------------------------------------------+
                              |
                              v (via ctypes / extern "C")
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

## 📂 Subsystem Definitions

### 1. High-Performance Core (`/hlv_core`)
Written in pure, standard C++17 with zero operating system dependencies:
* **`hlv_battery_core.hpp`**: Implements the dual-state $(\Psi, \Phi)$ system model. Physical attributes (voltage, current, temperature, state of charge) are coupled to informational attributes (entropy, history, degradation) using Marcel Krüger's effective metric formulation:
  $$g^{\text{eff}}_{\mu\nu} = g_{\mu\nu} + \lambda \partial_{\mu}\Phi \partial_{\nu}\Phi$$
* **`torque_enhancement.hpp`**: Translates battery health, entropy, and cell balances into intelligent real-time motor torque limits.
* **`hlv_regen_braking_manager_v1.hpp`**: Implements health-aware regenerative braking blending, allowing healthy cells to maximize energy recovery while protecting weak cells.

### 2. Python Ctypes & Fallback Wrappers (`/hlv_core/python`, `/hlv_core/python_fallback`)
* **`hlv_enhancer.py`**: A ctypes wrapper that loads the shared library and translates C structs into Python dictionaries. This permits seamless script-based telemetry parsing with no performance loss.
* **`hlv_enhancer_fallback.py`**: A pure-Python fallback approximating the exact physical equations. Runs on ultra-low-power systems where C++ toolchains are unavailable.

### 3. Verification & Diagnostics Scripts (`/scripts`)
Modular, robust diagnostics tools used during installation, updates, or maintenance:
* **`diagnostics.py`**: The primary diagnostic script checking OS compliance, Python versions, interface existence, and physical battery boundaries (SOH >= 80%, SOC 20-90%, Temperature 0-45°C).
* **`rollback.sh`**: Handles backup generation and state restores during setup and software updates.
* **`battery_health.sh` / `firmware_detect.sh` / `compatibility.sh`**: Light shell wrappers for targeted diagnostic views.

### 4. Configuration Layer (`/config`)
* **`vehicle_profiles.json`**: Supported EV models (including Tesla Model 3/Y/S/X, Hyundai Ioniq 5, Nissan Leaf e+) along with nominal voltages and firmware globs.
* **`safety_thresholds.json`**: Standard threshold definitions.
* **`install_policy.json`**: System-level installation rules and dependencies.
* **`vehicle_status.json`**: Mock state representing current simulated telemetries.

---

## 🔄 Lifecycle Workflows

### Installation Phase
1. The user invokes `make install`.
2. The `diagnostics.py` script is called to ensure safety.
3. If diagnostics pass, `rollback.sh` performs a backup of the existing `/opt/hlv_enhancement` files.
4. The C++ targets are compiled into `/hlv_core/bin` and `/hlv_core/lib`.
5. Files are cleanly copied to `/opt/hlv_enhancement/`.
6. Successful deployment is logged.

### Run-Time Phase
The vehicle control loop (100Hz) feeds physical sensor measurements (`voltage`, `current`, `temperature`, `soc`) into the HLV middleware:
1. `enhance_cycle(...)` updates the dual-state model.
2. If metric gradients indicate stress, torque limits are scaled back.
3. If cell voltage is unstable, regen braking blending is restricted.
4. The system updates logs and diagnostics continuously.
