# HLV BMS Middleware – OEM CAN Mapping Reference

**Version:** 1.0  
**Applies to:** `HLVHardwareAdapter`  
**Audience:** OEMs, Tier-1 suppliers, BMS / ECU integrators

---

## 1. Purpose

This document defines a **reference CAN signal mapping** used by the
HLV BMS Middleware hardware adapter.

It is **not a mandated DBC**.

Instead, it serves as:
- A clear integration template
- A safety reference
- A starting point for OEM-specific adaptation

OEMs are expected to:
- Map these signals to their existing DBC
- Adjust scaling and IDs as needed
- Preserve semantics and units

---

## 2. Design Principles

- **Safety first**: stale data detection is mandatory
- **Conservative defaults**: middleware fails closed
- **Minimal assumptions**: no vendor-specific encodings
- **Readable diffs**: stable signal names

---

## 3. Pack-Level Telemetry Signals (Required)

These signals **must** be provided for HLV middleware operation.

| Signal Name        | Description                    | Units | Direction | Required |
|--------------------|--------------------------------|-------|-----------|----------|
| PackVoltage        | Total pack voltage              | V     | HW → ECU  | Yes      |
| PackCurrent        | Pack current (signed)           | A     | HW → ECU  | Yes      |
| PackTemperature    | Representative pack temperature | °C    | HW → ECU  | Yes      |
| PackSOC            | Estimated state of charge       | %     | HW → ECU  | Yes      |

---

## 4. Reference CAN IDs (Example Only)

> ⚠️ **These IDs are placeholders.**  
> OEMs must replace them with values from their DBC.

| Signal          | CAN ID (hex) | Frame Type |
|-----------------|--------------|------------|
| PackVoltage     | 0x180        | Standard   |
| PackCurrent     | 0x181        | Standard   |
| PackTemperature | 0x182        | Standard   |
| PackSOC         | 0x183        | Standard   |

---

## 5. Reference Signal Encoding

### 5.1 Pack Voltage

- **Type:** Unsigned 16-bit
- **Endianness:** Little-endian
- **Scaling:**  
