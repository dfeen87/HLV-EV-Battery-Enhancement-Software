# HLV Intelligent Regenerative Braking  
## Architecture & OEM Integration Overview

**Module:** HLV Regen Braking Manager v1.0  
**Audience:** OEMs, Tier-1 suppliers, brake / chassis / controls engineers  
**Scope:** Regenerative braking limits, blending guidance, and safety cooperation  
**Status:** Production-lean, integration-ready

---

## 1. Purpose

This document describes the design and integration expectations of the **HLV Intelligent Regenerative Braking Manager**.

The module translates **battery-aware intelligence** into safe, stable regenerative braking limits while respecting:

- ABS / ESC authority  
- Brake-by-wire architectures  
- Battery safety constraints  
- Real-time control requirements  

It is designed to be **inserted into existing braking stacks**, not to replace them.

---

## 2. What This Module Does (and Does Not Do)

### ✔ What It Does
- Computes **maximum allowable regen torque** in real time
- Provides a **recommended regen fraction** for friction blending
- Protects battery health during regen charging
- Cooperates deterministically with ABS / ESC
- Smooths regen recovery after stability events
- Surfaces clear diagnostic reasons for regen limitation

### ✖ What It Does Not Do
- It does **not** compute wheel slip
- It does **not** replace ABS / ESC
- It does **not** command friction brakes
- It does **not** override OEM safety interlocks

The module **only limits and advises** regenerative braking.

---

## 3. High-Level Control Position



Brake Pedal
↓
Brake Request Mapping (OEM)
↓
┌────────────────────────────────────┐
│ HLV Regen Braking Manager │
│ - Battery acceptance limits │
│ - Stability cooperation │
│ - HLV stress awareness │
└───────────────▲────────────────────┘
│
Regen Torque Limit + Fraction
│
┌───────────────┴────────────────────┐
│ Brake Blending / BBW Controller │
│ - Regen applied up to limit │
│ - Friction fills remainder │
└───────────────▲────────────────────┘
│
ABS / ESC / Actuators


---

## 4. Inputs

### Required Inputs
- Enhanced battery state (`EnhancedState`)
- Brake request (normalized 0..1)
- Vehicle speed
- Time step (`dt`)

### Strongly Recommended
- `DiagnosticReport` from HLV BMS Middleware
- ABS active flag
- Wheel slip flag

### Optional
- Motor RPM or axle speed (for speed-dependent torque shaping)

---

## 5. Core Limiting Factors

The module computes regen limits using **multiplicative derating**, ensuring no single parameter dominates unsafely.

### 5.1 State of Charge (SOC)
- Regen tapers near high SOC
- Hard stop near SOC ceiling
- Prevents over-voltage and lithium plating

### 5.2 Pack Voltage Headroom
- Regen reduced as pack voltage approaches max
- Hard stop near absolute voltage limit

### 5.3 Temperature Acceptance
- Cold temperature regen reduction (plating protection)
- Hot temperature regen reduction (thermal stress control)

### 5.4 Cell Imbalance / Weak Cells
- Regen reduced when imbalance increases
- Prevents pushing weakest cells beyond safe limits

### 5.5 HLV Stress & Confidence
- Regen conservatively reduced when HLV confidence is low
- Allows long-term stress history to influence braking behavior

### 5.6 Stability Events (ABS / ESC)
- Immediate regen cut when ABS is active
- Partial cut when slip detected
- Smooth recovery after event clears

---

## 6. ABS / ESC Cooperation Model

### Hard Rules
- ABS / ESC **always win**
- Regen torque is cut immediately on ABS activation
- No oscillatory recovery after events

### Recovery Behavior
- Regen ramps back smoothly using a configurable time constant
- Prevents sudden rear-axle regen snap on low-µ surfaces

This behavior is critical for:
- Ice / snow
- Wet pavement
- Split-µ braking

---

## 7. Outputs

### Primary Outputs
- `max_regen_torque_nm`
- `regen_fraction` (0..1)

### Diagnostic Outputs
- Limiting factor identifier (SOC, voltage, temp, stability, etc.)
- Per-factor derate values
- Event counters (ABS, slip, safety blocks)

Diagnostics are intended for:
- Development validation
- Fleet telemetry
- Field issue analysis

---

## 8. Brake Blending Guidance (OEM)

A typical blending strategy:

Requested Brake Torque
│
├── Regen Torque = min(requested, max_regen_torque)
│
└── Friction Torque = remainder


Pedal feel should be handled **upstream** of this module to ensure consistency.

---

## 9. Safety Model

The regen manager:
- Fails closed on missing or invalid data
- Outputs zero regen on battery safety fault
- Never increases braking torque beyond driver request
- Never overrides ABS / ESC authority

It is intended to be **ASIL-compatible when integrated properly**, but it is not itself a certified safety controller.

---

## 10. Configuration Philosophy

Default parameters are:
- Conservative
- Chemistry-agnostic
- Safe across wide temperature ranges

OEMs are expected to:
- Tune thresholds per chemistry
- Validate behavior under fault injection
- Align regen limits with inverter capabilities

---

## 11. Summary

The HLV Regen Braking Manager provides a **battery-aware, stability-respecting, production-lean approach** to regenerative braking.

It extends HLV intelligence beyond propulsion into braking — where energy recovery, safety, and battery health intersect most critically.

This module completes the loop:
**Battery → Torque → Braking → Battery**

---

**Integration Support:**  
Don Michael Feeney Jr.  
📧 dfeen87@gmail.com
