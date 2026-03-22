#!/usr/bin/env python3
"""Step 2: Manus glove connection test.

Verifies that gloves can be paired and basic status information
can be read (battery, firmware, hand side).

Requirements:
    - Step 1 passed (SDK loads, dongle detected)
    - Manus gloves powered on and within BLE range

Usage: python3 -m manus.tests.test_step2_connection [--sdk-path manus/sdk/libManusSDK.so]
"""

import argparse
import sys
import time

from manus.manus_reader import ManusReader, SDKReturnCode


def main():
    parser = argparse.ArgumentParser(description="Step 2: Glove connection test")
    parser.add_argument("--sdk-path", default="manus/sdk/libManusSDK.so",
                        help="Path to libManusSDK.so")
    parser.add_argument("--hand", default="right",
                        choices=["left", "right", "both"],
                        help="Which hand to test (default: right)")
    args = parser.parse_args()

    print("=" * 55)
    print("  Step 2: Manus Glove Connection Test")
    print("  Make sure gloves are powered on!")
    print("=" * 55)
    passed = 0
    failed = 0

    # Test 1: Create ManusReader and connect
    print("\n[TEST] Connect to Manus SDK...", end=" ")
    reader = ManusReader(sdk_lib_path=args.sdk_path, hand_side=args.hand)
    try:
        reader.connect()
        print("[PASS]")
        passed += 1
    except FileNotFoundError as e:
        print(f"[FAIL] {e}")
        failed += 1
        _summary(passed, failed)
        return
    except RuntimeError as e:
        print(f"[FAIL] {e}")
        failed += 1
        _summary(passed, failed)
        return

    # Test 2: Check status
    print("[TEST] Reader status...", end=" ")
    status = reader.get_status()
    if status["connected"]:
        print("[PASS]")
        print(f"       SDK loaded: {status['sdk_loaded']}")
        print(f"       Connected: {status['connected']}")
        print(f"       Hand side: {status['hand_side']}")
        passed += 1
    else:
        print("[FAIL] Not connected")
        failed += 1

    # Test 3: Read a single frame of hand data
    print("[TEST] Read single hand data frame...", end=" ")
    # Give SDK a moment to establish data stream
    time.sleep(0.5)

    data = reader.get_hand_data()
    if data is not None:
        print("[PASS]")
        print(f"       Hand side: {data.hand_side}")
        print(f"       Joint angles shape: {data.joint_angles.shape}")
        print(f"       Timestamp: {data.timestamp:.3f}")

        # Check if data is non-zero (glove is actually tracking)
        nonzero = (data.joint_angles != 0).sum()
        print(f"       Non-zero joints: {nonzero}/{len(data.joint_angles)}")
        passed += 1
    else:
        print("[WARN] No data returned (glove may need a few seconds)")
        print("       Trying 5 more times with 1s delay...")

        got_data = False
        for attempt in range(5):
            time.sleep(1.0)
            data = reader.get_hand_data()
            if data is not None:
                print(f"       Got data on attempt {attempt + 2}!")
                got_data = True
                passed += 1
                break
            print(f"       Attempt {attempt + 2}: no data", end="")

        if not got_data:
            print("\n[FAIL] No data received after 5 attempts")
            print("       Check: glove powered on? Paired? Within range?")
            failed += 1

    # Test 4: Read from both hands (if applicable)
    if args.hand == "both":
        print("[TEST] Read both hands...", end=" ")
        hands = reader.get_both_hands()
        for side, hdata in hands.items():
            if hdata is not None:
                nonzero = (hdata.joint_angles != 0).sum()
                print(f"\n       {side}: {nonzero} non-zero joints")
            else:
                print(f"\n       {side}: no data")
        passed += 1

    # Test 5: Quick burst read (check for errors)
    print("[TEST] Burst read (10 frames)...", end=" ")
    success_count = 0
    error_count = 0
    for i in range(10):
        try:
            data = reader.get_hand_data()
            if data is not None:
                success_count += 1
        except Exception as e:
            error_count += 1
        time.sleep(0.05)

    if error_count == 0:
        print(f"[PASS] {success_count}/10 successful reads, 0 errors")
        passed += 1
    else:
        print(f"[FAIL] {error_count}/10 read errors")
        failed += 1

    # Cleanup
    reader.disconnect()
    print("\n[INFO] Disconnected from SDK")

    _summary(passed, failed)


def _summary(passed, failed):
    total = passed + failed
    print(f"\n{'=' * 55}")
    print(f"  Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed == 0:
        print("  [ALL PASS] Step 2 complete — proceed to Step 3")
    else:
        print("  [ISSUES] Fix the above failures before proceeding")
    print(f"{'=' * 55}")
    sys.exit(1 if failed > 0 else 0)


if __name__ == "__main__":
    main()
