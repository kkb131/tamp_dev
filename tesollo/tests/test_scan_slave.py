#!/usr/bin/env python3
"""Scan Modbus slave IDs to find the DG5F hand's unit ID.

Tries read_input_registers on slave IDs 0-247 and reports which ones respond.

Usage:
    python3 -m tesollo.tests.test_scan_slave --ip 169.254.186.72
    python3 -m tesollo.tests.test_scan_slave --ip 169.254.186.72 --start 0 --end 10
"""

import argparse
import sys

try:
    from pymodbus.client import ModbusTcpClient
except ImportError:
    print("pymodbus required: pip install 'pymodbus>=3.10,<4'")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Scan Modbus slave IDs for DG5F")
    parser.add_argument("--ip", default="169.254.186.72", help="DG5F IP address")
    parser.add_argument("--port", type=int, default=502, help="Modbus TCP port")
    parser.add_argument("--start", type=int, default=0, help="Start slave ID (default: 0)")
    parser.add_argument("--end", type=int, default=247, help="End slave ID (default: 247)")
    parser.add_argument("--timeout", type=float, default=0.5, help="Timeout per ID (seconds)")
    args = parser.parse_args()

    print(f"Scanning slave IDs {args.start}-{args.end} on {args.ip}:{args.port}")
    print(f"Timeout per ID: {args.timeout}s")
    print("-" * 50)

    client = ModbusTcpClient(host=args.ip, port=args.port, timeout=args.timeout)
    if not client.connect():
        print(f"[FAIL] Cannot connect to {args.ip}:{args.port}")
        sys.exit(1)

    found = []
    for sid in range(args.start, args.end + 1):
        result = client.read_input_registers(0, count=1, device_id=sid)
        if not result.isError():
            print(f"  [HIT] slave ID = {sid}  (reg[0] = {result.registers[0]})")
            found.append(sid)
        elif sid % 50 == 0:
            print(f"  ... scanning ID {sid} ...")

    client.close()

    print("-" * 50)
    if found:
        print(f"Found {len(found)} responding slave ID(s): {found}")
        print(f"\nUse:  --slave-id {found[0]}")
    else:
        print("No responding slave IDs found.")
        print("Check: hand power, network cable, IP address.")


if __name__ == "__main__":
    main()
