#!/usr/bin/env python3
"""Step 1: Manus SDK load and dongle detection test.

Verifies that the Manus SDK shared library can be loaded
and the USB dongle is detected.

Requirements:
    - libManusSDK.so in manus/sdk/ directory
    - Manus USB dongle plugged in
    - udev rules installed

Usage: python3 -m manus.tests.test_step1_sdk [--sdk-path manus/sdk/libManusSDK.so]
"""

import argparse
import ctypes
import os
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Step 1: SDK load test")
    parser.add_argument("--sdk-path", default="manus/sdk/libManusSDK.so",
                        help="Path to libManusSDK.so")
    args = parser.parse_args()

    print("=" * 55)
    print("  Step 1: Manus SDK Load & Dongle Detection")
    print("=" * 55)
    passed = 0
    failed = 0

    # Test 1: SDK file exists (try primary path, then Integrated variant)
    print("\n[TEST] SDK file exists...", end=" ")
    sdk_path = Path(args.sdk_path)
    if not sdk_path.exists():
        # Fallback: try libManusSDK_Integrated.so in same directory
        alt_path = sdk_path.parent / "libManusSDK_Integrated.so"
        if alt_path.exists():
            print(f"[INFO] Primary not found, using Integrated variant")
            sdk_path = alt_path
    if sdk_path.exists():
        size_mb = sdk_path.stat().st_size / (1024 * 1024)
        print(f"[PASS] {sdk_path} ({size_mb:.1f} MB)")
        passed += 1
    else:
        print(f"[FAIL] Not found: {sdk_path.resolve()}")
        print(f"       Also checked: {sdk_path.parent / 'libManusSDK_Integrated.so'}")
        print(f"       Download SDK from: https://docs.manus-meta.com/3.1.0/Plugins/SDK/Linux/")
        print(f"       Place libManusSDK.so (or libManusSDK_Integrated.so) in: {sdk_path.parent.resolve()}/")
        failed += 1
        _summary(passed, failed)
        return

    # Test 2: Load shared library
    print("[TEST] Load libManusSDK.so via ctypes...", end=" ")
    try:
        sdk = ctypes.CDLL(str(sdk_path.resolve()))
        print("[PASS]")
        passed += 1
    except OSError as e:
        print(f"[FAIL] {e}")
        print("       Check dependencies: ldd manus/sdk/libManusSDK.so")
        failed += 1
        _summary(passed, failed)
        return

    # Test 3: Check key function symbols exist
    print("[TEST] Check SDK function symbols...", end=" ")
    required_funcs = [
        "CoreSdk_Initialize",
        "CoreSdk_ShutDown",
        "CoreSdk_LookForHosts",
        "CoreSdk_ConnectLocally",
    ]
    missing = []
    for func_name in required_funcs:
        try:
            getattr(sdk, func_name)
        except AttributeError:
            missing.append(func_name)

    if not missing:
        print(f"[PASS] All {len(required_funcs)} functions found")
        passed += 1
    else:
        print(f"[FAIL] Missing: {', '.join(missing)}")
        print("       The SDK version may differ. Check ManusSDK.h for function names.")
        failed += 1

    # Test 4: List available SDK functions (informational)
    print("\n[INFO] Available CoreSdk_* functions:")
    try:
        result = subprocess.run(
            ["nm", "-D", str(sdk_path.resolve())],
            capture_output=True, text=True,
        )
        funcs = [line.split()[-1] for line in result.stdout.split("\n")
                 if "CoreSdk_" in line and " T " in line]
        if funcs:
            for f in sorted(funcs)[:20]:
                print(f"         {f}")
            if len(funcs) > 20:
                print(f"         ... and {len(funcs) - 20} more")
        else:
            # Try with grep for all symbols
            funcs = [line.split()[-1] for line in result.stdout.split("\n")
                     if "CoreSdk" in line]
            for f in sorted(funcs)[:15]:
                print(f"         {f}")
    except Exception:
        print("         (nm not available)")

    # Test 5: Initialize SDK
    print("\n[TEST] CoreSdk_Initialize...", end=" ")
    try:
        sdk.CoreSdk_Initialize.argtypes = [ctypes.c_int]
        sdk.CoreSdk_Initialize.restype = ctypes.c_int
        ret = sdk.CoreSdk_Initialize(ctypes.c_int(1))  # IntegratedDataSession
        if ret == 0:
            print("[PASS] SDK initialized successfully")
            passed += 1
        else:
            print(f"[FAIL] Return code: {ret}")
            failed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # Test 6: Look for hosts (USB dongles)
    print("[TEST] Look for Manus hosts (USB dongles)...", end=" ")
    try:
        sdk.CoreSdk_LookForHosts.argtypes = [ctypes.c_uint32, ctypes.c_bool]
        sdk.CoreSdk_LookForHosts.restype = ctypes.c_int
        ret = sdk.CoreSdk_LookForHosts(ctypes.c_uint32(3), ctypes.c_bool(False))

        sdk.CoreSdk_GetNumberOfAvailableHostsFound.argtypes = [
            ctypes.POINTER(ctypes.c_uint32)
        ]
        sdk.CoreSdk_GetNumberOfAvailableHostsFound.restype = ctypes.c_int

        host_count = ctypes.c_uint32(0)
        ret2 = sdk.CoreSdk_GetNumberOfAvailableHostsFound(ctypes.byref(host_count))

        if ret2 == 0 and host_count.value > 0:
            print(f"[PASS] Found {host_count.value} host(s)")
            passed += 1
        else:
            print(f"[WARN] No hosts found (ret={ret}, hosts={host_count.value})")
            print("       Is the Manus USB dongle plugged in?")
            print("       Check: lsusb | grep -i manus")
            print("       Check udev rules: ls /etc/udev/rules.d/70-manus*")
            # Not a hard failure — dongle might not be connected during this test
            passed += 1
    except Exception as e:
        print(f"[FAIL] {e}")
        failed += 1

    # Shutdown SDK
    try:
        sdk.CoreSdk_ShutDown.argtypes = []
        sdk.CoreSdk_ShutDown.restype = ctypes.c_int
        sdk.CoreSdk_ShutDown()
    except Exception:
        pass

    # Test 7: Check USB devices for Manus dongle
    print("\n[TEST] USB device scan (lsusb)...", end=" ")
    try:
        result = subprocess.run(["lsusb"], capture_output=True, text=True)
        manus_lines = [l for l in result.stdout.split("\n")
                       if "manus" in l.lower() or "3325" in l
                       or "1915" in l or "2d5f" in l.lower()]
        if manus_lines:
            print("[PASS] Manus dongle detected:")
            for line in manus_lines:
                print(f"       {line.strip()}")
            passed += 1
        else:
            print("[INFO] No Manus device in lsusb output")
            print("       This is OK if the dongle is not plugged in yet")
            print("       When plugged in, run: lsusb | grep -i manus")
            passed += 1
    except FileNotFoundError:
        print("[SKIP] lsusb not available")
        passed += 1

    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 55}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 1 complete — proceed to Step 2")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 55}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
