#!/usr/bin/env python3
"""SDK 심볼 자동 탐색 + 코드 패치 스크립트.

Manus SDK v3.1.0에서 C API가 변경된 경우,
라이브러리 심볼 또는 헤더 파서 리포트를 기반으로 코드를 자동 패치합니다.

Usage:
    cd ~/tamp_ws/src/tamp_dev

    # 방법 1: nm -D 기반 (단순 이름 변경만)
    python3 -m manus.fix_sdk_symbols --sdk-path manus/sdk/libManusSDK.so
    python3 -m manus.fix_sdk_symbols --sdk-path manus/sdk/libManusSDK.so --apply

    # 방법 2: 헤더 파서 리포트 기반 (API 구조 변경 대응)
    python3 -m manus.parse_sdk_headers --sdk-dir manus/sdk/   # 먼저 리포트 생성
    python3 -m manus.fix_sdk_symbols --from-report manus/sdk/api_report.json
    python3 -m manus.fix_sdk_symbols --from-report manus/sdk/api_report.json --apply
"""

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


# 현재 코드에서 사용하는 CoreSdk 함수 목록
EXPECTED_FUNCTIONS = [
    "CoreSdk_Initialize",
    "CoreSdk_ShutDown",
    "CoreSdk_LookForHosts",
    "CoreSdk_GetNumberOfAvailableHostsFound",
    "CoreSdk_ConnectToHost",
    "CoreSdk_ConnectLocally",
    "CoreSdk_GetRawErgonomicsData",
]

# 패치 대상 파일 (스크립트 위치 기준 상대 경로)
PATCH_TARGETS = [
    "manus_reader.py",
    "tests/test_step1_sdk.py",
]


# ── nm -D 기반 심볼 탐색 ──────────────────────────────

def discover_symbols(sdk_path: str) -> list[str]:
    """nm -D 로 SDK에서 CoreSdk 관련 심볼을 모두 추출."""
    try:
        result = subprocess.run(
            ["nm", "-D", sdk_path],
            capture_output=True, text=True, timeout=10,
        )
    except FileNotFoundError:
        print("[ERROR] 'nm' 명령을 찾을 수 없습니다. binutils를 설치하세요:")
        print("        sudo apt install binutils")
        sys.exit(1)

    if result.returncode != 0:
        print(f"[ERROR] nm 실행 실패: {result.stderr.strip()}")
        sys.exit(1)

    symbols = []
    for line in result.stdout.split("\n"):
        parts = line.strip().split()
        if len(parts) >= 3 and parts[1] == "T":
            sym = parts[2]
            if "coresdk" in sym.lower():
                symbols.append(sym)
        elif len(parts) >= 2 and parts[0] == "T":
            sym = parts[1]
            if "coresdk" in sym.lower():
                symbols.append(sym)

    return sorted(set(symbols))


def match_functions(expected: list[str], actual: list[str]) -> dict[str, str | None]:
    """기대 함수명 → 실제 SDK 심볼 매핑 (case-insensitive fuzzy match)."""
    mapping = {}
    actual_lower = {s.lower(): s for s in actual}

    for func in expected:
        func_lower = func.lower()

        # 1) 정확히 일치
        if func in actual:
            mapping[func] = func
            continue

        # 2) 대소문자 무시 일치
        if func_lower in actual_lower:
            mapping[func] = actual_lower[func_lower]
            continue

        # 3) 키워드 기반 매칭
        keyword = func.replace("CoreSdk_", "").lower()
        candidates = []
        for sym in actual:
            sym_keyword = sym.lower().replace("coresdk_", "").replace("coresdk", "")
            if keyword.replace("_", "") == sym_keyword.replace("_", ""):
                candidates.append(sym)

        if len(candidates) == 1:
            mapping[func] = candidates[0]
        elif len(candidates) > 1:
            mapping[func] = candidates[0]
        else:
            mapping[func] = None

    return mapping


# ── 리포트 기반 패치 ──────────────────────────────────

def patch_from_report(report_path: Path, manus_dir: Path, dry_run: bool = True):
    """api_report.json 기반으로 manus_reader.py를 패치."""
    report = json.loads(report_path.read_text())
    func_names = {f["name"] for f in report["functions"]}
    func_map = {f["name"]: f for f in report["functions"]}

    print(f"\n  SDK 함수 {len(func_names)}개 발견")

    # ── 1. 함수 이름 매핑 ──
    rename_map = {}  # old_name → new_name

    # Initialize → InitializeIntegrated
    if "CoreSdk_Initialize" not in func_names:
        if "CoreSdk_InitializeIntegrated" in func_names:
            rename_map["CoreSdk_Initialize"] = "CoreSdk_InitializeIntegrated"

    # ConnectLocally → 삭제 (특수 처리)
    connect_locally_removed = "CoreSdk_ConnectLocally" not in func_names

    # GetRawErgonomicsData → 대체 함수 탐색
    ergo_replacement = None
    if "CoreSdk_GetRawErgonomicsData" not in func_names:
        # 우선순위: GetRawDeviceData > GetRawSkeletonData
        for candidate in ["CoreSdk_GetRawDeviceData",
                          "CoreSdk_GetRawSkeletonData"]:
            if candidate in func_names:
                ergo_replacement = candidate
                break

    # ── 매핑 리포트 출력 ──
    print(f"\n  {'기존 함수':<45} {'새 함수':<45} 상태")
    print(f"  {'-'*45} {'-'*45} {'-'*8}")

    for func in EXPECTED_FUNCTIONS:
        if func in rename_map:
            print(f"  {func:<45} {rename_map[func]:<45} RENAME")
        elif func == "CoreSdk_ConnectLocally" and connect_locally_removed:
            print(f"  {func:<45} {'(삭제됨 — fallback 제거)':<45} REMOVED")
        elif func == "CoreSdk_GetRawErgonomicsData" and ergo_replacement:
            print(f"  {func:<45} {ergo_replacement:<45} REPLACE")
        elif func in func_names:
            print(f"  {func:<45} {func:<45} OK")
        else:
            print(f"  {func:<45} {'??? (매칭 실패)':<45} MISS")

    # ── InitializeIntegrated 시그니처 확인 ──
    init_func = func_map.get("CoreSdk_InitializeIntegrated")
    init_has_params = init_func and init_func["param_count"] > 0

    # ── 대체 데이터 함수 시그니처 ──
    ergo_func = func_map.get(ergo_replacement) if ergo_replacement else None

    # ── Ergonomics 관련 struct 탐색 ──
    ergo_structs = {}
    for s in report.get("structs", []):
        name_lower = s["name"].lower()
        if any(k in name_lower for k in ["ergonomic", "device", "glove",
                                          "finger", "hand"]):
            ergo_structs[s["name"]] = s

    # ── 2. 패치 실행 ──
    reader_path = manus_dir / "manus_reader.py"
    test_path = manus_dir / "tests/test_step1_sdk.py"

    if not reader_path.exists():
        print(f"[ERROR] {reader_path} 없음")
        return

    content = reader_path.read_text()
    new_content = content

    # 2a. Initialize 패치
    if "CoreSdk_Initialize" in rename_map:
        new_name = rename_map["CoreSdk_Initialize"]
        new_content = new_content.replace("CoreSdk_Initialize", new_name)
        if not init_has_params:
            # 파라미터 제거: InitializeIntegrated(c_int(1)) → InitializeIntegrated()
            new_content = new_content.replace(
                f"{new_name}(ctypes.c_int(1))",
                f"{new_name}()",
            )
            # 시그니처도 업데이트
            new_content = new_content.replace(
                f"sdk.{new_name}.argtypes = [ctypes.c_int]",
                f"sdk.{new_name}.argtypes = []",
            )

    # 2b. ConnectLocally fallback 제거
    if connect_locally_removed:
        # ConnectLocally fallback 블록을 에러로 대체
        old_fallback = """        else:
            # Try direct local connection
            ret = self._sdk.CoreSdk_ConnectLocally()
            if ret == SDKReturnCode.SUCCESS:
                self._connected = True
                print("[ManusReader] Connected locally")
            else:
                raise RuntimeError(
                    f"CoreSdk_ConnectLocally failed with code {ret}. "
                    "No Manus dongle found. Check USB connection and udev rules."
                )"""
        new_fallback = """        else:
            raise RuntimeError(
                "No Manus hosts found. Check USB dongle connection and udev rules."
            )"""
        new_content = new_content.replace(old_fallback, new_fallback)

        # 시그니처 제거
        old_sig = """
        # CoreSdk_ConnectLocally()
        sdk.CoreSdk_ConnectLocally.argtypes = []
        sdk.CoreSdk_ConnectLocally.restype = ctypes.c_int"""
        new_content = new_content.replace(old_sig, "")

    # 2c. GetRawErgonomicsData → 대체 함수
    if ergo_replacement:
        new_content = new_content.replace(
            "CoreSdk_GetRawErgonomicsData", ergo_replacement
        )

    # ── 변경사항 출력 ──
    if new_content == content:
        print(f"\n[SKIP] manus_reader.py — 변경 없음")
    elif dry_run:
        print(f"\n[DRY-RUN] manus_reader.py에서 다음 변경이 적용될 예정:")
        if "CoreSdk_Initialize" in rename_map:
            new_name = rename_map["CoreSdk_Initialize"]
            count = content.count("CoreSdk_Initialize")
            params_info = "(파라미터 제거)" if not init_has_params else ""
            print(f"  CoreSdk_Initialize → {new_name} ({count}개소) {params_info}")
        if connect_locally_removed:
            print(f"  CoreSdk_ConnectLocally fallback 블록 제거")
        if ergo_replacement:
            count = content.count("CoreSdk_GetRawErgonomicsData")
            print(f"  CoreSdk_GetRawErgonomicsData → {ergo_replacement} ({count}개소)")
    else:
        backup = reader_path.with_suffix(".py.bak")
        shutil.copy2(reader_path, backup)
        print(f"[BACKUP] {backup}")
        reader_path.write_text(new_content)
        print(f"[PATCH]  manus_reader.py 패치 완료")

    # ── test_step1_sdk.py 패치 ──
    if test_path.exists():
        test_content = test_path.read_text()
        new_test = test_content
        for old_name, new_name in rename_map.items():
            new_test = new_test.replace(old_name, new_name)
        if connect_locally_removed:
            # required_funcs 리스트에서 ConnectLocally 제거
            new_test = new_test.replace(
                '        "CoreSdk_ConnectLocally",\n', ''
            )
        if ergo_replacement:
            new_test = new_test.replace(
                "CoreSdk_GetRawErgonomicsData", ergo_replacement
            )

        if new_test == test_content:
            print(f"[SKIP] tests/test_step1_sdk.py — 변경 없음")
        elif dry_run:
            print(f"[DRY-RUN] tests/test_step1_sdk.py에서도 동일 변경 적용 예정")
        else:
            backup = test_path.with_suffix(".py.bak")
            shutil.copy2(test_path, backup)
            print(f"[BACKUP] {backup}")
            test_path.write_text(new_test)
            print(f"[PATCH]  tests/test_step1_sdk.py 패치 완료")

    # ── 수동 확인 필요 사항 ──
    print(f"\n{'=' * 60}")
    print(f"  수동 확인 필요 사항")
    print(f"{'=' * 60}")

    if ergo_replacement and ergo_func:
        params = ", ".join(p["raw"] for p in ergo_func["params"])
        print(f"\n  데이터 함수 시그니처:")
        print(f"    {ergo_func['return_type']} {ergo_replacement}({params})")
        print(f"\n    ⚠ ErgonomicsStream/ErgonomicsData struct가 변경되었을 수 있습니다.")
        print(f"      api_report.json에서 관련 struct를 확인하고")
        print(f"      manus_reader.py의 ctypes struct 정의를 수동으로 업데이트하세요.")

    if ergo_structs:
        print(f"\n  관련 struct ({len(ergo_structs)}개):")
        for name, s in ergo_structs.items():
            print(f"\n    struct {name} {{")
            for field in s["fields"]:
                arr = f'[{field["array_size"]}]' if "array_size" in field else ""
                print(f"      {field['type']} {field['name']}{arr};")
            print(f"    }}")

    if not dry_run:
        print(f"\n[DONE] 패치 완료.")
        print(f"       ⚠ struct 정의는 수동 확인 필요!")
        print(f"       테스트: python3 -m manus.tests.test_step1_sdk")


# ── nm 기반 패치 (기존 로직) ──────────────────────────

def apply_patches_nm(mapping: dict[str, str], manus_dir: Path, dry_run: bool = True):
    """매핑에 따라 파일을 패치 (단순 이름 치환)."""
    changes = {old: new for old, new in mapping.items()
               if new is not None and old != new}

    if not changes:
        print("\n[INFO] 변경이 필요한 함수가 없습니다.")
        return

    for rel_path in PATCH_TARGETS:
        filepath = manus_dir / rel_path
        if not filepath.exists():
            print(f"[WARN] 파일 없음: {filepath}")
            continue

        content = filepath.read_text()
        new_content = content
        for old_name, new_name in changes.items():
            if old_name in new_content:
                new_content = new_content.replace(old_name, new_name)

        if new_content == content:
            print(f"[SKIP] {rel_path} — 변경 없음")
            continue

        if dry_run:
            print(f"\n[DRY-RUN] {rel_path}에서 다음 치환이 적용될 예정:")
            for old_name, new_name in changes.items():
                count = content.count(old_name)
                if count > 0:
                    print(f"           {old_name} → {new_name}  ({count}개소)")
        else:
            backup = filepath.with_suffix(filepath.suffix + ".bak")
            shutil.copy2(filepath, backup)
            print(f"[BACKUP] {backup}")
            filepath.write_text(new_content)
            for old_name, new_name in changes.items():
                count = content.count(old_name)
                if count > 0:
                    print(f"[PATCH]  {rel_path}: {old_name} → {new_name}  ({count}개소)")


def main():
    parser = argparse.ArgumentParser(
        description="Manus SDK 심볼 탐색 + 코드 자동 패치",
    )
    parser.add_argument(
        "--sdk-path", default="manus/sdk/libManusSDK.so",
        help="SDK .so 파일 경로 (nm -D 모드)",
    )
    parser.add_argument(
        "--from-report", default=None,
        help="api_report.json 경로 (헤더 파서 리포트 모드)",
    )
    parser.add_argument(
        "--apply", action="store_true",
        help="실제로 파일을 패치 (기본: dry-run)",
    )
    args = parser.parse_args()

    manus_dir = Path(__file__).parent

    # ── 리포트 기반 모드 ──
    if args.from_report:
        report_path = Path(args.from_report)
        if not report_path.exists():
            print(f"[ERROR] 리포트 파일 없음: {report_path}")
            print(f"        먼저 실행: python3 -m manus.parse_sdk_headers --sdk-dir manus/sdk/")
            sys.exit(1)

        print("=" * 60)
        print("  Manus SDK 코드 패치 (헤더 리포트 기반)")
        print(f"  리포트: {report_path}")
        print(f"  모드: {'APPLY (실제 패치)' if args.apply else 'DRY-RUN (확인만)'}")
        print("=" * 60)

        patch_from_report(report_path, manus_dir, dry_run=not args.apply)
        return

    # ── nm -D 기반 모드 (기존) ──
    sdk_path = Path(args.sdk_path)

    if not sdk_path.exists():
        alt = sdk_path.parent / "libManusSDK_Integrated.so"
        if alt.exists():
            print(f"[INFO] {sdk_path} 없음 → Integrated 버전 사용: {alt}")
            sdk_path = alt
        else:
            print(f"[ERROR] SDK 파일을 찾을 수 없습니다:")
            print(f"        {sdk_path}")
            print(f"        {alt}")
            sys.exit(1)

    print("=" * 60)
    print("  Manus SDK 심볼 탐색 + 코드 패치 (nm -D 모드)")
    print(f"  SDK: {sdk_path}")
    print(f"  모드: {'APPLY (실제 패치)' if args.apply else 'DRY-RUN (확인만)'}")
    print("=" * 60)

    print(f"\n[1/3] SDK 심볼 탐색 중...")
    actual_symbols = discover_symbols(str(sdk_path))

    if not actual_symbols:
        print("[ERROR] CoreSdk 관련 심볼을 찾을 수 없습니다.")
        print(f"        nm -D {sdk_path} | head -20")
        sys.exit(1)

    print(f"  발견된 CoreSdk 심볼 ({len(actual_symbols)}개):")
    for sym in actual_symbols:
        print(f"    {sym}")

    print(f"\n[2/3] 함수 매핑 중...")
    mapping = match_functions(EXPECTED_FUNCTIONS, actual_symbols)

    print(f"\n  {'우리 코드':<45} {'SDK 심볼':<45} 상태")
    print(f"  {'-'*45} {'-'*45} {'-'*6}")
    all_matched = True
    for old, new in mapping.items():
        if new is None:
            status = "MISS"
            all_matched = False
            print(f"  {old:<45} {'??? (매칭 실패)':<45} {status}")
        elif old == new:
            print(f"  {old:<45} {new:<45} OK")
        else:
            print(f"  {old:<45} {new:<45} RENAME")

    if not all_matched:
        print("\n[WARN] 일부 함수 매핑에 실패했습니다.")
        print("       SDK API가 크게 변경된 경우 헤더 파서 모드를 사용하세요:")
        print("       python3 -m manus.parse_sdk_headers --sdk-dir manus/sdk/")
        print("       python3 -m manus.fix_sdk_symbols --from-report manus/sdk/api_report.json")

    print(f"\n[3/3] 코드 패치...")
    apply_patches_nm(mapping, manus_dir, dry_run=not args.apply)

    if not args.apply:
        rename_count = sum(1 for old, new in mapping.items()
                          if new is not None and old != new)
        if rename_count > 0:
            print(f"\n[INFO] {rename_count}개 함수 이름 변경이 필요합니다.")
            print(f"       python3 -m manus.fix_sdk_symbols --sdk-path {sdk_path} --apply")

    if args.apply:
        print(f"\n[DONE] 패치 완료.")
        print(f"       python3 -m manus.tests.test_step1_sdk --sdk-path {sdk_path}")


if __name__ == "__main__":
    main()
