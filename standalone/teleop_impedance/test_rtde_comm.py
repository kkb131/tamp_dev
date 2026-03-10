#!/usr/bin/env python3
"""RTDE communication diagnostic for UR10e impedance torque control.

Tests each RTDE capability individually to identify what works and what fails.
Run on a machine connected to the robot:

  python3 -m standalone.teleop_impedance.test_rtde_comm --robot-ip 192.168.0.2

Results are printed as PASS/FAIL for each test.
"""

import argparse
import sys
import time
from pathlib import Path

_SCRIPT_PATH = Path(__file__).parent / "scripts" / "impedance_pd.script"


def test_recv_interface(ip: str) -> bool:
    """Test 1: RTDEReceiveInterface - read robot state."""
    print("\n=== TEST 1: RTDEReceiveInterface (robot state) ===")
    try:
        import rtde_receive
        recv = rtde_receive.RTDEReceiveInterface(ip)
        q = list(recv.getActualQ())
        qd = list(recv.getActualQd())
        tcp = list(recv.getActualTCPPose())
        print(f"  q  = [{', '.join(f'{v:.4f}' for v in q)}]")
        print(f"  qd = [{', '.join(f'{v:.4f}' for v in qd)}]")
        print(f"  tcp= [{', '.join(f'{v:.4f}' for v in tcp)}]")
        recv.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_ctrl_interface(ip: str, freq: float = 500.0) -> bool:
    """Test 2: RTDEControlInterface - basic connection."""
    print(f"\n=== TEST 2: RTDEControlInterface (freq={freq}Hz) ===")
    try:
        import rtde_control
        ctrl = rtde_control.RTDEControlInterface(ip, freq)
        connected = ctrl.isConnected()
        print(f"  connected = {connected}")
        ctrl.stopScript()
        ctrl.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_ctrl_dynamics(ip: str, freq: float = 500.0) -> bool:
    """Test 3: RTDEControlInterface dynamics queries (Coriolis, joint torques)."""
    print(f"\n=== TEST 3: RTDEControlInterface dynamics queries ===")
    try:
        import rtde_receive
        import rtde_control
        recv = rtde_receive.RTDEReceiveInterface(ip)
        ctrl = rtde_control.RTDEControlInterface(ip, freq)

        q = list(recv.getActualQ())
        qd = list(recv.getActualQd())

        coriolis = list(ctrl.getCoriolisAndCentrifugalTorques(q, qd))
        print(f"  coriolis = [{', '.join(f'{v:.4f}' for v in coriolis)}]")

        jtorques = list(ctrl.getJointTorques())
        print(f"  joint_torques = [{', '.join(f'{v:.4f}' for v in jtorques)}]")

        ctrl.stopScript()
        ctrl.disconnect()
        recv.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_io_lower(ip: str) -> bool:
    """Test 4: RTDEIOInterface lower range - double + int register writes."""
    print("\n=== TEST 4: RTDEIOInterface (lower range, double+int) ===")
    try:
        import rtde_io
        io = rtde_io.RTDEIOInterface(ip, use_upper_range_registers=False)

        # Write double registers 18-22
        for i in range(18, 23):
            io.setInputDoubleRegister(i, 0.0)
        print("  double regs 18-22: write OK")

        # Write int registers 18-19
        io.setInputIntRegister(18, 0)
        io.setInputIntRegister(19, 0)
        print("  int regs 18-19: write OK")

        io.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_io_upper(ip: str) -> bool:
    """Test 5: RTDEIOInterface upper range (often fails due to register conflict)."""
    print("\n=== TEST 5: RTDEIOInterface (upper range) ===")
    try:
        import rtde_io
        io = rtde_io.RTDEIOInterface(ip, use_upper_range_registers=True)
        io.setInputDoubleRegister(42, 0.0)
        print("  double reg 42: write OK")
        io.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_send_script_file(ip: str, freq: float = 500.0) -> bool:
    """Test 6: sendCustomScriptFile(path) - upload URScript."""
    print(f"\n=== TEST 6: sendCustomScriptFile(path) ===")
    try:
        import rtde_control
        ctrl = rtde_control.RTDEControlInterface(ip, freq)

        if not _SCRIPT_PATH.exists():
            print(f"  SKIP: script not found at {_SCRIPT_PATH}")
            ctrl.disconnect()
            return False

        result = ctrl.sendCustomScriptFile(str(_SCRIPT_PATH))
        print(f"  sendCustomScriptFile(path) returned: {result}")

        ctrl.stopScript()
        ctrl.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_send_script_function(ip: str, freq: float = 500.0) -> bool:
    """Test 7: sendCustomScript(code_string) - upload URScript as string."""
    print(f"\n=== TEST 7: sendCustomScript(code_string) ===")
    try:
        import rtde_control
        ctrl = rtde_control.RTDEControlInterface(ip, freq)

        test_script = 'def test_prog():\n  textmsg("hello")\n  sync()\nend\ntest_prog()\n'
        result = ctrl.sendCustomScript(test_script)
        print(f"  sendCustomScript(str) returned: {result}")

        time.sleep(0.5)
        ctrl.stopScript()
        ctrl.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_init_wait_period(ip: str, freq: float = 500.0) -> bool:
    """Test 8: initPeriod/waitPeriod timing control."""
    print(f"\n=== TEST 8: initPeriod / waitPeriod (timing) ===")
    try:
        import rtde_control
        ctrl = rtde_control.RTDEControlInterface(ip, freq)

        t0 = ctrl.initPeriod()
        ctrl.waitPeriod(t0)
        print("  initPeriod + waitPeriod: OK")

        ctrl.stopScript()
        ctrl.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_direct_torque_builtin(ip: str, freq: float = 500.0) -> bool:
    """Test 9: ur_rtde built-in directTorque() (known buggy in 1.6.2)."""
    print(f"\n=== TEST 9: directTorque() built-in (may be buggy) ===")
    try:
        import rtde_control
        ctrl = rtde_control.RTDEControlInterface(ip, freq)

        if not hasattr(ctrl, 'directTorque'):
            print("  SKIP: directTorque not available in this ur_rtde version")
            ctrl.disconnect()
            return False

        # Try sending zero torque briefly
        zero_tau = [0.0] * 6
        t0 = ctrl.initPeriod()
        result = ctrl.directTorque(zero_tau)
        ctrl.waitPeriod(t0)
        print(f"  directTorque([0]*6) returned: {result}")

        ctrl.stopScript()
        ctrl.disconnect()
        print("  PASS (but check UR teach pendant for Int type error!)")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


def test_full_pipeline(ip: str, freq: float = 500.0) -> bool:
    """Test 10: Full pipeline - recv + ctrl + io + script upload + register write."""
    print(f"\n=== TEST 10: Full pipeline (recv + ctrl + io + script + regs) ===")
    try:
        import rtde_receive
        import rtde_control
        import rtde_io

        # Step 1: recv
        recv = rtde_receive.RTDEReceiveInterface(ip)
        q = list(recv.getActualQ())
        print(f"  recv: OK (q[0]={q[0]:.4f})")

        # Step 2: ctrl
        ctrl = rtde_control.RTDEControlInterface(ip, freq)
        print(f"  ctrl: OK (connected={ctrl.isConnected()})")

        # Step 3: io (lower range)
        io = rtde_io.RTDEIOInterface(ip, use_upper_range_registers=False)
        for i in range(18, 23):
            io.setInputDoubleRegister(i, 0.0)
        io.setInputIntRegister(18, 0)
        io.setInputIntRegister(19, 0)  # mode=idle
        print("  io: OK (regs zeroed)")

        # Step 4: upload script
        if _SCRIPT_PATH.exists():
            result = ctrl.sendCustomScriptFile(str(_SCRIPT_PATH))
            print(f"  script upload: returned {result}")
        else:
            print(f"  script upload: SKIP (file not found)")

        # Step 5: activate mode
        io.setInputIntRegister(19, 1)  # mode=active
        print("  mode=active: OK")

        # Step 6: write test torques (all zero)
        for i in range(18, 23):
            io.setInputDoubleRegister(i, 0.0)
        io.setInputIntRegister(18, 0)  # tau5=0
        print("  zero torque write: OK")

        time.sleep(1.0)

        # Step 7: stop
        io.setInputIntRegister(19, -1)  # mode=stop
        print("  mode=stop: OK")

        time.sleep(0.2)
        ctrl.stopScript()
        ctrl.disconnect()
        recv.disconnect()
        io.disconnect()
        print("  PASS")
        return True
    except Exception as e:
        print(f"  FAIL at step: {e}")
        # Cleanup
        for obj_name in ['io', 'ctrl', 'recv']:
            try:
                obj = locals().get(obj_name)
                if obj:
                    obj.disconnect()
            except Exception:
                pass
        return False


def main():
    parser = argparse.ArgumentParser(description="RTDE communication diagnostic")
    parser.add_argument("--robot-ip", required=True, help="Robot IP address")
    parser.add_argument("--freq", type=float, default=500.0, help="Control frequency")
    parser.add_argument("--test", type=int, default=0,
                        help="Run specific test (1-10), 0=all")
    args = parser.parse_args()

    tests = [
        (1, test_recv_interface),
        (2, test_ctrl_interface),
        (3, test_ctrl_dynamics),
        (4, test_io_lower),
        (5, test_io_upper),
        (6, test_send_script_file),
        (7, test_send_script_function),
        (8, test_init_wait_period),
        (9, test_direct_torque_builtin),
        (10, test_full_pipeline),
    ]

    results = {}
    for num, fn in tests:
        if args.test > 0 and num != args.test:
            continue
        try:
            results[num] = fn(args.robot_ip, args.freq) if 'freq' in fn.__code__.co_varnames else fn(args.robot_ip)
        except Exception as e:
            print(f"  UNEXPECTED ERROR: {e}")
            results[num] = False

    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    for num, fn in tests:
        if num in results:
            status = "PASS" if results[num] else "FAIL"
            print(f"  Test {num:2d}: {status}  {fn.__doc__.strip().split(chr(10))[0]}")

    failed = [n for n, r in results.items() if not r]
    if failed:
        print(f"\nFailed tests: {failed}")
    else:
        print("\nAll tests passed!")


if __name__ == "__main__":
    main()
