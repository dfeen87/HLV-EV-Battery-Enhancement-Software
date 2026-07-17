# HLV EV Battery Enhancement Software - Installation Guide

Welcome to the installation and deployment manual for the Helix-Light-Vortex (HLV) EV Battery Enhancement Software. This system deploys advanced dual-state (Ψ, Φ) physics-based algorithms directly alongside your vehicle's existing Battery Management System (BMS) to optimize battery life, fast charging profiles, and regenerative braking.

---

## 📋 Table of Contents
- [Prerequisites](#prerequisites)
- [System Requirements](#system-requirements)
- [Target Directory Layout](#target-directory-layout)
- [Workflow Instructions](#workflow-instructions)
  - [1. Verification](#1-verification)
  - [2. Pre-Flight Diagnostics](#2-pre-flight-diagnostics)
  - [3. System Installation](#3-system-installation)
  - [4. Software Updates](#4-software-updates)
  - [5. Backup & Rollback](#5-backup--rollback)
- [Manual Configuration Overrides](#manual-configuration-overrides)
- [Troubleshooting & Logs](#troubleshooting--logs)

---

## 🔧 Prerequisites

Before starting, ensure that the vehicle computer has:
1. **Operating System**: A POSIX-compliant system (Ubuntu Linux 20.04+ or macOS 12+ recommended).
2. **C++ Compiler**: A compiler supporting **C++17** or newer (`g++ >= 9` or `clang >= 11`).
3. **Build Tools**: `make` and `cmake` (version 3.15+).
4. **Python**: **Python >= 3.8** with standard library modules.

---

## 💻 System Requirements

The HLV software stack has a lightweight footprint suitable for automotive ECU-adjacent integration:
* **Memory**: ~50KB per battery pack instance.
* **CPU**: <3% overhead at 100Hz BMS loop update frequency.
* **Execution Latency**: sub-100µs (C++ core).

---

## 📂 Target Directory Layout

When installed, the package is deployed to the system root under:
`/opt/hlv_enhancement/`

The directories are organized as follows:
* `/opt/hlv_enhancement/bin` — Compiled high-performance C++ CLI tool (`hlv_enhancer`).
* `/opt/hlv_enhancement/lib` — Shared library files (`libhlv_enhancer.so` or `libhlv_enhancer.dylib`).
* `/opt/hlv_enhancement/python` — Python `ctypes` bindings for C++ library integrations.
* `/opt/hlv_enhancement/python_fallback` — Pure-Python mathematical fallback module.
* `/opt/hlv_enhancement/config` — Vehicle profiles, safety thresholds, and installation policies.
* `/opt/hlv_enhancement/logs` — Comprehensive log files including `install.log`.
* `/opt/hlv_enhancement/backups` — Timestamped, historical system rollback states.

---

## 🚀 Workflow Instructions

The complete setup workflow is fully driven by a top-level `Makefile`.

### 1. Verification
Run system pre-requisite environment checks (checking target OS, compiler capabilities, Python versions, and write permissions):
```bash
make verify
```

### 2. Pre-Flight Diagnostics
Run comprehensive, safety-enforced vehicle diagnostics (validates SOH, SOC, temperatures, and vehicle profile firmware matches):
```bash
make diagnostics
```
*Note: This must pass for `make install` to proceed.*

### 3. System Installation
Installs the complete package to `/opt/hlv_enhancement`. This requires administrative privileges (via `sudo`) to write to `/opt` and configure system paths:
```bash
sudo make install
```
This target automatically:
1. Runs the validation checks first.
2. Backs up any existing previous installations under `/opt/hlv_enhancement/backups/`.
3. Compiles the C++ core engine (`hlv_enhancer`) and shared libraries.
4. deploys binary, config, script, python, and documentation files to `/opt/hlv_enhancement`.
5. Logs all outcomes and decisions to `/opt/hlv_enhancement/logs/install.log`.

### 4. Software Updates
Pulls the latest code version, re-validates safety and compatibility checks, compiles any new updates, and deploys them:
```bash
sudo make update
```

### 5. Backup & Rollback
If any unexpected behavior occurs after an update, revert immediately to the previous stable backup using:
```bash
sudo make rollback
```
This restores the most recent timestamped backup state and updates the log.

---

## ⚙️ Manual Configuration Overrides

By default, diagnostics query `/tmp/vehicle_status.json` or `/config/vehicle_status.json`. You can simulate various physical vehicle conditions during verification using:

### CLI Overrides
```bash
# Force diagnostics to evaluate with a low battery health (this will fail)
./scripts/diagnostics.py --soh 74.0

# Force diagnostics to evaluate with a high temperature condition (this will fail)
./scripts/diagnostics.py --temp 55.0
```

### Environment Variable Overrides
```bash
# Override State of Charge
export HLV_SOC=85.0
make diagnostics
```

---

## 📝 Troubleshooting & Logs

All diagnostic decisions, warnings, and system alterations are securely written to the following log file:
`/opt/hlv_enhancement/logs/install.log`

If you encounter a build failure:
1. Ensure `cmake` is fully installed.
2. Delete the temporary build cache (`rm -rf build/`) and retry.
3. Check that your user has `sudo` privileges.
