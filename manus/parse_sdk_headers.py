#!/usr/bin/env python3
"""Manus SDK 헤더 파서 — ManusSDK.h / ManusSDKTypes.h 분석.

SDK 디렉토리의 .h 파일을 파싱하여 함수 시그니처, struct, enum 정의를 추출하고
JSON 리포트를 생성합니다. 이 리포트를 fix_sdk_symbols.py에서 사용하여
manus_reader.py를 자동 패치합니다.

Usage:
    cd ~/tamp_ws/src/tamp_dev
    python3 -m manus.parse_sdk_headers --sdk-dir manus/sdk/
    # → manus/sdk/api_report.json 생성
"""

import argparse
import json
import re
import sys
from pathlib import Path


# ── C type → ctypes 매핑 ──────────────────────────────

C_TO_CTYPES = {
    "void": "None",
    "bool": "ctypes.c_bool",
    "int": "ctypes.c_int",
    "int32_t": "ctypes.c_int32",
    "uint32_t": "ctypes.c_uint32",
    "int64_t": "ctypes.c_int64",
    "uint64_t": "ctypes.c_uint64",
    "float": "ctypes.c_float",
    "double": "ctypes.c_double",
    "char": "ctypes.c_char",
    "size_t": "ctypes.c_size_t",
    "SDKReturnCode": "ctypes.c_int",
    "SessionType": "ctypes.c_int",
    "Side": "ctypes.c_uint32",
}


def parse_functions(header_text: str) -> list[dict]:
    """Extract CoreSdk_* function declarations from header text."""
    functions = []

    # Pattern: return_type CoreSdk_FuncName(params);
    # Handles multi-line declarations and various return type formats
    # Also handles CORESDK_API or __declspec(dllexport) prefixes
    pattern = re.compile(
        r'(?:CORESDK_API|__declspec\s*\(\s*dllexport\s*\)|extern\s+"C")?\s*'
        r'(\w+)\s+'                          # return type
        r'(CoreSdk_\w+)\s*'                  # function name
        r'\(([^)]*)\)\s*;',                  # parameters
        re.MULTILINE | re.DOTALL,
    )

    for m in pattern.finditer(header_text):
        ret_type = m.group(1).strip()
        func_name = m.group(2).strip()
        params_raw = m.group(3).strip()

        params = []
        if params_raw and params_raw != "void":
            for p in params_raw.split(","):
                p = p.strip()
                if not p:
                    continue
                # Handle "const Type* name", "Type name", "Type* name"
                p = re.sub(r'\s+', ' ', p)
                parts = p.rsplit(" ", 1)
                if len(parts) == 2:
                    ptype = parts[0].strip()
                    pname = parts[1].strip().lstrip("*")
                    is_pointer = "*" in parts[0] or parts[1].startswith("*")
                    params.append({
                        "type": ptype.replace("*", "").strip(),
                        "name": pname,
                        "is_pointer": is_pointer,
                        "is_const": "const" in ptype,
                        "raw": p,
                    })
                else:
                    params.append({"type": p, "name": "?", "is_pointer": "*" in p,
                                   "is_const": "const" in p, "raw": p})

        functions.append({
            "name": func_name,
            "return_type": ret_type,
            "params": params,
            "param_count": len(params),
        })

    return sorted(functions, key=lambda f: f["name"])


def parse_structs(header_text: str) -> list[dict]:
    """Extract struct definitions from header text."""
    structs = []

    # Pattern 1: typedef struct { ... } Name;
    pattern1 = re.compile(
        r'typedef\s+struct\s*(?:\w+)?\s*\{([^}]+(?:\{[^}]*\}[^}]*)*)\}\s*(\w+)\s*;',
        re.MULTILINE | re.DOTALL,
    )
    for m in pattern1.finditer(header_text):
        body = m.group(1)
        name = m.group(2).strip()
        fields = _parse_struct_fields(body)
        structs.append({"name": name, "fields": fields})

    # Pattern 2: struct Name { ... };
    pattern2 = re.compile(
        r'struct\s+(\w+)\s*\{([^}]+(?:\{[^}]*\}[^}]*)*)\}\s*;',
        re.MULTILINE | re.DOTALL,
    )
    for m in pattern2.finditer(header_text):
        name = m.group(1).strip()
        body = m.group(2)
        # Skip if already found via typedef
        if any(s["name"] == name for s in structs):
            continue
        fields = _parse_struct_fields(body)
        structs.append({"name": name, "fields": fields})

    return sorted(structs, key=lambda s: s["name"])


def _parse_struct_fields(body: str) -> list[dict]:
    """Parse fields from struct body text."""
    fields = []
    for line in body.split(";"):
        line = line.strip()
        if not line or line.startswith("//") or line.startswith("/*"):
            continue
        # Remove comments
        line = re.sub(r'//.*$', '', line).strip()
        line = re.sub(r'/\*.*?\*/', '', line).strip()
        if not line:
            continue

        # Handle array: "Type name[SIZE]"
        arr_match = re.match(r'(.+?)\s+(\w+)\s*\[(\w+)\]', line)
        if arr_match:
            ftype = arr_match.group(1).strip()
            fname = arr_match.group(2).strip()
            fsize = arr_match.group(3).strip()
            fields.append({"type": ftype, "name": fname, "array_size": fsize})
            continue

        # Handle pointer: "Type* name"
        ptr_match = re.match(r'(.+?)\s*\*\s*(\w+)', line)
        if ptr_match:
            ftype = ptr_match.group(1).strip()
            fname = ptr_match.group(2).strip()
            fields.append({"type": ftype + "*", "name": fname})
            continue

        # Simple: "Type name"
        parts = line.rsplit(None, 1)
        if len(parts) == 2:
            fields.append({"type": parts[0].strip(), "name": parts[1].strip()})
        elif parts:
            fields.append({"type": line, "name": "?"})

    return fields


def parse_enums(header_text: str) -> list[dict]:
    """Extract enum definitions from header text."""
    enums = []

    # typedef enum { ... } Name;
    pattern = re.compile(
        r'typedef\s+enum\s*(?:\w+)?\s*\{([^}]+)\}\s*(\w+)\s*;',
        re.MULTILINE | re.DOTALL,
    )
    for m in pattern.finditer(header_text):
        body = m.group(1)
        name = m.group(2).strip()
        values = _parse_enum_values(body)
        enums.append({"name": name, "values": values})

    return sorted(enums, key=lambda e: e["name"])


def _parse_enum_values(body: str) -> list[dict]:
    """Parse enum values from body text."""
    values = []
    counter = 0
    for line in body.split(","):
        line = re.sub(r'//.*$', '', line, flags=re.MULTILINE).strip()
        line = re.sub(r'/\*.*?\*/', '', line).strip()
        if not line:
            continue
        if "=" in line:
            parts = line.split("=", 1)
            name = parts[0].strip()
            try:
                val = int(parts[1].strip().rstrip(","))
                counter = val + 1
            except ValueError:
                val = parts[1].strip()
                counter += 1
            values.append({"name": name, "value": val})
        else:
            values.append({"name": line.strip().rstrip(","), "value": counter})
            counter += 1
    return values


def parse_defines(header_text: str) -> list[dict]:
    """Extract #define constants."""
    defines = []
    for m in re.finditer(r'#define\s+(\w+)\s+(\d+)', header_text):
        defines.append({"name": m.group(1), "value": int(m.group(2))})
    return defines


def generate_migration_advice(report: dict) -> list[str]:
    """Generate migration advice based on parsed API report."""
    advice = []
    func_names = {f["name"] for f in report["functions"]}

    # 1. Initialize
    if "CoreSdk_InitializeIntegrated" in func_names:
        func = next(f for f in report["functions"]
                     if f["name"] == "CoreSdk_InitializeIntegrated")
        if func["param_count"] == 0:
            advice.append(
                "CoreSdk_Initialize(c_int(1)) → CoreSdk_InitializeIntegrated()  "
                "[파라미터 제거]"
            )
        else:
            params = ", ".join(p["raw"] for p in func["params"])
            advice.append(
                f"CoreSdk_Initialize(c_int(1)) → CoreSdk_InitializeIntegrated({params})  "
                f"[파라미터 {func['param_count']}개]"
            )

    # 2. ConnectLocally
    if "CoreSdk_ConnectLocally" not in func_names:
        advice.append(
            "CoreSdk_ConnectLocally() → 삭제됨. "
            "ConnectToHost fallback 제거 필요."
        )

    # 3. GetRawErgonomicsData
    if "CoreSdk_GetRawErgonomicsData" not in func_names:
        # Find replacement candidates
        candidates = [f for f in report["functions"]
                      if "GetRaw" in f["name"] or "Ergonomic" in f["name"]
                      or "GetDevice" in f["name"]]
        if candidates:
            names = ", ".join(f["name"] for f in candidates)
            advice.append(
                f"CoreSdk_GetRawErgonomicsData → 삭제됨. "
                f"후보: {names}"
            )
            for c in candidates:
                params = ", ".join(f'{p["type"]} {p["name"]}' for p in c["params"])
                advice.append(f"  {c['return_type']} {c['name']}({params})")
        else:
            advice.append(
                "CoreSdk_GetRawErgonomicsData → 삭제됨. "
                "대체 함수를 찾을 수 없음!"
            )

    # 4. Ergonomics structs
    ergo_structs = [s for s in report["structs"]
                    if "ergonomic" in s["name"].lower()
                    or "device" in s["name"].lower()
                    or "skeleton" in s["name"].lower()]
    if ergo_structs:
        advice.append("\n관련 struct 정의:")
        for s in ergo_structs:
            fields_str = ", ".join(
                f'{f["type"]} {f["name"]}'
                + (f'[{f["array_size"]}]' if "array_size" in f else "")
                for f in s["fields"]
            )
            advice.append(f"  {s['name']} {{ {fields_str} }}")

    # 5. Side enum
    side_enums = [e for e in report["enums"] if "side" in e["name"].lower()]
    if side_enums:
        for e in side_enums:
            vals = ", ".join(f'{v["name"]}={v["value"]}' for v in e["values"])
            advice.append(f"\nSide enum: {e['name']} {{ {vals} }}")

    return advice


def main():
    parser = argparse.ArgumentParser(
        description="Manus SDK 헤더 파서 — API 리포트 생성",
    )
    parser.add_argument(
        "--sdk-dir", default="manus/sdk",
        help="SDK 디렉토리 경로 (ManusSDK.h가 있는 곳)",
    )
    parser.add_argument(
        "--output", default=None,
        help="JSON 리포트 출력 경로 (기본: <sdk-dir>/api_report.json)",
    )
    args = parser.parse_args()

    sdk_dir = Path(args.sdk_dir)
    if not sdk_dir.exists():
        print(f"[ERROR] SDK 디렉토리를 찾을 수 없습니다: {sdk_dir}")
        sys.exit(1)

    # Find header files
    headers = list(sdk_dir.glob("*.h")) + list(sdk_dir.glob("**/*.h"))
    if not headers:
        print(f"[ERROR] .h 파일을 찾을 수 없습니다: {sdk_dir}")
        print("        SDK 압축을 해제했는지 확인하세요.")
        sys.exit(1)

    print("=" * 60)
    print("  Manus SDK 헤더 파서")
    print(f"  SDK 디렉토리: {sdk_dir}")
    print("=" * 60)

    # Read all headers
    all_text = ""
    for h in sorted(headers):
        print(f"  [READ] {h.name} ({h.stat().st_size} bytes)")
        all_text += "\n" + h.read_text(errors="replace")

    # Parse
    print(f"\n[1/4] 함수 파싱...")
    functions = parse_functions(all_text)
    coresdk_funcs = [f for f in functions if f["name"].startswith("CoreSdk_")]
    print(f"  CoreSdk_* 함수: {len(coresdk_funcs)}개")

    print(f"[2/4] struct 파싱...")
    structs = parse_structs(all_text)
    print(f"  struct 정의: {len(structs)}개")

    print(f"[3/4] enum 파싱...")
    enums = parse_enums(all_text)
    print(f"  enum 정의: {len(enums)}개")

    print(f"[4/4] #define 파싱...")
    defines = parse_defines(all_text)
    print(f"  #define 상수: {len(defines)}개")

    # Build report
    report = {
        "sdk_dir": str(sdk_dir),
        "headers": [str(h) for h in headers],
        "functions": coresdk_funcs,
        "structs": structs,
        "enums": enums,
        "defines": defines,
    }

    # Output JSON
    output_path = Path(args.output) if args.output else sdk_dir / "api_report.json"
    output_path.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\n[SAVED] {output_path}")

    # Print summary
    print(f"\n{'=' * 60}")
    print(f"  CoreSdk_* 함수 목록 ({len(coresdk_funcs)}개)")
    print(f"{'=' * 60}")
    for f in coresdk_funcs:
        params = ", ".join(p["raw"] for p in f["params"])
        print(f"  {f['return_type']} {f['name']}({params})")

    # Migration advice
    print(f"\n{'=' * 60}")
    print(f"  v2.4.0 → v3.1.0 마이그레이션 제안")
    print(f"{'=' * 60}")
    advice = generate_migration_advice(report)
    for line in advice:
        print(f"  {line}")

    # Ergonomics-related info
    print(f"\n{'=' * 60}")
    print(f"  Ergonomics / 핑거 데이터 관련 struct")
    print(f"{'=' * 60}")
    keywords = ["ergonomic", "finger", "hand", "device", "glove", "skeleton"]
    related = [s for s in structs
               if any(k in s["name"].lower() for k in keywords)]
    if related:
        for s in related:
            print(f"\n  struct {s['name']} {{")
            for field in s["fields"]:
                arr = f'[{field["array_size"]}]' if "array_size" in field else ""
                print(f"    {field['type']} {field['name']}{arr};")
            print(f"  }}")
    else:
        print("  (관련 struct를 찾을 수 없음 — 헤더 파일을 직접 확인하세요)")

    print(f"\n[DONE] 리포트 저장됨: {output_path}")
    print(f"       다음 단계:")
    print(f"       1. 위 출력에서 데이터 함수/struct 매핑 확인")
    print(f"       2. api_report.json을 git commit")
    print(f"       3. dev PC에서 fix_sdk_symbols.py --from-report로 패치")


if __name__ == "__main__":
    main()
