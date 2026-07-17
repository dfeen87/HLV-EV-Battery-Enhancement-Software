# -*- coding: utf-8 -*-
"""
HLV EV Battery Enhancement - Python ctypes Wrapper (Option B)
=============================================================

This module provides a pure-Python wrapper around the high-performance
compiled C++17 HLV middleware and physics engine.
"""

import os
import sys
import ctypes

# Define the C-compatible struct matching hlv_core/src/hlv_enhancer.cpp
class HLVResult(ctypes.Structure):
    _fields_ = [
        ("degradation", ctypes.c_double),
        ("remaining_capacity_percent", ctypes.c_double),
        ("cycles_to_80_percent", ctypes.c_double),
        ("hlv_confidence", ctypes.c_double),
        ("recommended_current_limit", ctypes.c_double),
        ("recommended_voltage_limit", ctypes.c_double),
        ("recommended_temperature", ctypes.c_double),
        ("estimated_charge_time", ctypes.c_double),
        ("degradation_warning", ctypes.c_int),
        ("input_clamped", ctypes.c_int),
        ("numerical_stability", ctypes.c_int)
    ]

class HLVEnhancer:
    def __init__(self, capacity_ah=75.0, nominal_voltage_v=400.0, lib_path=None):
        self.capacity_ah = capacity_ah
        self.nominal_voltage_v = nominal_voltage_v
        self.lib = None
        self.handle = None

        if lib_path is None:
            # Look in standard build/installation paths
            possible_paths = [
                # In-tree locations
                os.path.join(os.path.dirname(__file__), "../lib/libhlv_enhancer.so"),
                os.path.join(os.path.dirname(__file__), "../lib/libhlv_enhancer.dylib"),
                # Installed location
                "/opt/hlv_enhancement/lib/libhlv_enhancer.so",
                "/opt/hlv_enhancement/lib/libhlv_enhancer.dylib",
                # System locations
                "/usr/local/lib/libhlv_enhancer.so",
                "/usr/lib/libhlv_enhancer.so"
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    lib_path = path
                    break

        if lib_path is None or not os.path.exists(lib_path):
            raise FileNotFoundError(
                "Could not find compiled shared library libhlv_enhancer.so. "
                "Ensure that the HLV C++ code is built and installed properly."
            )

        # Load library
        self.lib = ctypes.CDLL(lib_path)

        # Declare C-function signatures
        self.lib.hlv_create.restype = ctypes.c_void_p
        self.lib.hlv_create.argtypes = [ctypes.c_double, ctypes.c_double]

        self.lib.hlv_destroy.restype = None
        self.lib.hlv_destroy.argtypes = [ctypes.c_void_p]

        self.lib.hlv_enhance.restype = ctypes.c_int
        self.lib.hlv_enhance.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(HLVResult)
        ]

        # Initialize HLV instance
        self.handle = self.lib.hlv_create(self.capacity_ah, self.nominal_voltage_v)
        if not self.handle:
            raise RuntimeError("Failed to initialize HLV Middleware instance in shared library.")

    def enhance_cycle(self, voltage, current, temperature, soc, dt=0.1):
        """
        Updates the HLV engine with physical readings and returns the enhanced predictions.
        """
        if not self.handle:
            raise RuntimeError("Enhancer instance has been destroyed.")

        result = HLVResult()
        ret = self.lib.hlv_enhance(
            self.handle,
            float(voltage),
            float(current),
            float(temperature),
            float(soc),
            float(dt),
            ctypes.byref(result)
        )

        if ret != 0:
            raise RuntimeError(f"HLV Core evaluation failed with code {ret}")

        return {
            "degradation": result.degradation,
            "remaining_capacity_percent": result.remaining_capacity_percent,
            "cycles_to_80_percent": result.cycles_to_80_percent,
            "hlv_confidence": result.hlv_confidence,
            "recommended_current_limit": result.recommended_current_limit,
            "recommended_voltage_limit": result.recommended_voltage_limit,
            "recommended_temperature": result.recommended_temperature,
            "estimated_charge_time": result.estimated_charge_time,
            "degradation_warning": bool(result.degradation_warning),
            "input_clamped": bool(result.input_clamped),
            "numerical_stability": bool(result.numerical_stability)
        }

    def __del__(self):
        if self.lib and self.handle:
            self.lib.hlv_destroy(self.handle)
            self.handle = None

# Simple self-test code when run directly
if __name__ == "__main__":
    print("Running HLV Python Wrapper Self-Test...")
    try:
        enhancer = HLVEnhancer(capacity_ah=75.0, nominal_voltage_v=400.0)
        res = enhancer.enhance_cycle(voltage=355.0, current=50.0, temperature=25.0, soc=0.65, dt=0.1)
        print("Self-test SUCCESS! Results:")
        for k, v in res.items():
            print(f"  {k}: {v}")
    except Exception as e:
        print(f"Self-test failed: {e}")
        sys.exit(1)
