# Manus SDK 설치

이 디렉토리에 Manus SDK for Linux 파일을 배치하세요.

## 다운로드

1. Manus 개발자 포털에서 SDK 다운로드:
   - https://docs.manus-meta.com/2.4.0/Plugins/SDK/Linux/
   - "Download SDK" → Linux 버전 선택

2. 다운로드한 아카이브를 이 디렉토리에 압축 해제

## 필요한 파일

압축 해제 후 다음 파일이 이 디렉토리에 있어야 합니다:

```
sdk/
├── libManusSDK.so              # 핵심 공유 라이브러리
├── ManusSDK.h                  # C API 헤더
├── ManusSDKTypes.h             # 타입 정의 헤더
└── (기타 SDK 파일들)
```

## 확인

```bash
# 라이브러리 파일 확인
ls -la manus/sdk/libManusSDK.so

# 심볼 확인
nm -D manus/sdk/libManusSDK.so | grep -i "CoreSdk"
```

## 주의사항

- SDK 파일은 `.gitignore`에 추가되어 있습니다 (라이선스 제약)
- 각 테스트 PC에서 별도로 다운로드/배치해야 합니다
- SDK 버전에 따라 `manus_reader.py`의 ctypes 매핑 업데이트가 필요할 수 있습니다
