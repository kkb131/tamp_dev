#!/usr/bin/env python3
"""Step 1: Manus SDK v3.1.0 load and dongle detection test.

Verifies that the Manus SDK shared library can be loaded
and key v3.1.0 function symbols exist.

Requirements:
    - libManusSDK.so or libManusSDK_Integrated.so in manus/sdk/ directory
    - Manus USB dongle plugged in (optional for symbol check)

Usage: python3 -m manus.tests.test_step1_sdk [--sdk-path manus/sdk/libManusSDK.so]
"""

import argparse
import ctypes
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Step 1: SDK v3.1.0 load test")
    parser.add_argument("--sdk-path", default="manus/sdk/libManusSDK.so",
                        help="Path to libManusSDK.so")
    args = parser.parse_args()

    print("=" * 55)
    print("  Step 1: Manus SDK v3.1.0 Load & Symbol Check")
    print("=" * 55)
    passed = 0
    failed = 0

    # Test 1: SDK file exists (try primary, then Integrated variant)
    print("\n[TEST] SDK file exists...", end=" ")
    sdk_path = Path(args.sdk_path)
    if not sdk_path.exists():
        alt_path = sdk_path.parent / "libManusSDK_Integrated.so"
        if alt_path.exists():
            print(f"[INFO] Using Integrated variant")
            sdk_path = alt_path
    if sdk_path.exists():
        size_mb = sdk_path.stat().st_size / (1024 * 1024)
        print(f"[PASS] {sdk_path} ({size_mb:.1f} MB)")
        passed += 1
    else:
        print(f"[FAIL] Not found: {sdk_path.resolve()}")
        print(f"       Also checked: {sdk_path.parent / 'libManusSDK_Integrated.so'}")
        print(f"       Download from: https://docs.manus-meta.com/3.1.0/Plugins/SDK/Linux/")
        failed += 1
        _summary(passed, failed)
        return

    # Test 2: Load shared library
    print("[TEST] Load SDK via ctypes...", end=" ")
    try:
        sdk = ctypes.CDLL(str(sdk_path.resolve()))
        print("[PASS]")
        passed += 1
    except OSError as e:
        print(f"[FAIL] {e}")
        print("       Check dependencies: ldd " + str(sdk_path))
        failed += 1
        _summary(passed, failed)
        return

    # Test 3: Check v3.1.0 function symbols
    print("[TEST] Check SDK v3.1.0 function symbols...", end=" ")
    required_funcs = [
        "CoreSdk_InitializeIntegrated",
        "CoreSdk_ShutDown",
        "CoreSdk_InitializeCoordinateSystemWithVUH",
        "CoreSdk_RegisterCallbackForErgonomicsStream",
        "CoreSdk_ConnectToHost",
        "CoreSdk_LookForHosts",
        "CoreSdk_GetNumberOfAvailableHostsFound",
        "CoreSdk_GetNumberOfDongles",
        "CoreSdk_GetGlovesForDongle",
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

    # Test 4: List available CoreSdk_* functions (informational)
    print("\n[INFO] Available CoreSdk_* functions:")
    try:
        result = subprocess.run(
            ["nm", "-D", str(sdk_path.resolve())],
            capture_output=True, text=True,
        )
        funcs = [line.split()[-1] for line in result.stdout.split("\n")
                 if "CoreSdk_" in line and " T " in line]
        if funcs:
            for f in sorted(funcs)[:25]:
                print(f"         {f}")
            if len(funcs) > 25:
                print(f"         ... and {len(funcs) - 25} more")
        else:
            funcs = [line.split()[-1] for line in result.stdout.split("\n")
                     if "CoreSdk" in line]
            for f in sorted(funcs)[:15]:
                print(f"         {f}")
    except Exception:
        print("         (nm not available)")

    # Test 5: Initialize SDK (Integrated)
    print("\n[TEST] CoreSdk_InitializeIntegrated...", end=" ")
    try:
        sdk.CoreSdk_InitializeIntegrated.argtypes = []
        sdk.CoreSdk_InitializeIntegrated.restype = ctypes.c_int
        ret = sdk.CoreSdk_InitializeIntegrated()
        if ret == 0:
            print("[PASS] SDK initialized successfully")
            passed += 1
        else:
            print(f"[FAIL] Return code: {ret}")
            failed += 1
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

    # Test 6: Check USB devices for Manus dongle
    print("\n[TEST] USB device scan (lsusb)...", end=" ")
    try:
        result = subprocess.run(["lsusb"], capture_output=True, text=True)
        manus_lines = [l for l in result.stdout.split("\n")
                       if "manus" in l.lower() or "3325" in l
                       or "1915" in l]
        if manus_lines:
            print("[PASS] Manus device detected:")
            for line in manus_lines:
                print(f"       {line.strip()}")
            passed += 1
        else:
            print("[INFO] No Manus device in lsusb output")
            print("       This is OK if the dongle is not plugged in yet")
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
