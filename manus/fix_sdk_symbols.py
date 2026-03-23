#!/usr/bin/env python3
"""SDK 심볼 자동 탐색 + 코드 패치 스크립트.

Manus SDK v3.1.0에서 C API 함수 이름이 변경된 경우,
실제 라이브러리의 심볼을 탐색하여 코드를 자동으로 패치합니다.

Usage:
    cd ~/tamp_ws/src/tamp_dev

    # 1. dry-run (매핑만 확인)
    python3 -m manus.fix_sdk_symbols --sdk-path manus/sdk/libManusSDK.so

    # 2. 매핑이 맞으면 적용
    python3 -m manus.fix_sdk_symbols --sdk-path manus/sdk/libManusSDK.so --apply
"""

import argparse
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

        # 3) 키워드 기반 매칭 (예: "Initialize" → "initialize")
        # 함수명에서 CoreSdk_ 접두사 제거 후 비교
        keyword = func.replace("CoreSdk_", "").lower()
        candidates = []
        for sym in actual:
            sym_keyword = sym.lower().replace("coresdk_", "").replace("coresdk", "")
            # 언더스코어 제거 후 비교
            if keyword.replace("_", "") == sym_keyword.replace("_", ""):
                candidates.append(sym)

        if len(candidates) == 1:
            mapping[func] = candidates[0]
        elif len(candidates) > 1:
            # 여러 후보가 있으면 가장 비슷한 것 선택
            mapping[func] = candidates[0]
        else:
            mapping[func] = None

    return mapping


def apply_patches(mapping: dict[str, str], manus_dir: Path, dry_run: bool = True):
    """매핑에 따라 파일을 패치."""
    # 변경이 필요한 매핑만 필터
    changes = {old: new for old, new in mapping.items()
               if new is not None and old != new}

    if not changes:
        print("\n[INFO] 변경이 필요한 함수가 없습니다. 모든 심볼이 이미 일치합니다.")
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
            # 백업 생성
            backup = filepath.with_suffix(filepath.suffix + ".bak")
            shutil.copy2(filepath, backup)
            print(f"[BACKUP] {backup}")

            # 패치 적용
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
        help="SDK .so 파일 경로 (기본: manus/sdk/libManusSDK.so)",
    )
    parser.add_argument(
        "--apply", action="store_true",
        help="실제로 파일을 패치 (기본: dry-run)",
    )
    args = parser.parse_args()

    sdk_path = Path(args.sdk_path)

    # Integrated 버전 fallback
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
    print("  Manus SDK 심볼 탐색 + 코드 패치")
    print(f"  SDK: {sdk_path}")
    print(f"  모드: {'APPLY (실제 패치)' if args.apply else 'DRY-RUN (확인만)'}")
    print("=" * 60)

    # 1. 심볼 탐색
    print(f"\n[1/3] SDK 심볼 탐색 중...")
    actual_symbols = discover_symbols(str(sdk_path))

    if not actual_symbols:
        print("[ERROR] CoreSdk 관련 심볼을 찾을 수 없습니다.")
        print("        SDK 파일이 올바른지 확인하세요:")
        print(f"        nm -D {sdk_path} | head -20")
        sys.exit(1)

    print(f"  발견된 CoreSdk 심볼 ({len(actual_symbols)}개):")
    for sym in actual_symbols:
        print(f"    {sym}")

    # 2. 매핑
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
            status = "OK"
            print(f"  {old:<45} {new:<45} {status}")
        else:
            status = "RENAME"
            print(f"  {old:<45} {new:<45} {status}")

    if not all_matched:
        print("\n[WARN] 일부 함수 매핑에 실패했습니다.")
        print("       SDK에서 해당 함수가 제거되었거나 이름이 크게 변경되었을 수 있습니다.")
        print("       위 '발견된 CoreSdk 심볼' 목록에서 수동으로 확인하세요.")

    # 3. 패치
    print(f"\n[3/3] 코드 패치...")
    manus_dir = Path(__file__).parent
    apply_patches(mapping, manus_dir, dry_run=not args.apply)

    if not args.apply:
        rename_count = sum(1 for old, new in mapping.items()
                          if new is not None and old != new)
        if rename_count > 0:
            print(f"\n[INFO] {rename_count}개 함수 이름 변경이 필요합니다.")
            print(f"       적용하려면 --apply 플래그를 추가하세요:")
            print(f"       python3 -m manus.fix_sdk_symbols --sdk-path {sdk_path} --apply")
        else:
            print("\n[INFO] 모든 함수 이름이 일치합니다. 패치 불필요.")

    if args.apply:
        print("\n[DONE] 패치 완료. 테스트를 실행하세요:")
        print(f"       python3 -m manus.tests.test_step1_sdk --sdk-path {sdk_path}")


if __name__ == "__main__":
    main()
