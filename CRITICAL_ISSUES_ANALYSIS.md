# 성능 저하 원인 - 심각한 추가 문제 발견

## 🔴 가장 심각한 문제: LightGlue가 일부분에서만 사용됨

### OKVIS2 매칭 구조 분석

OKVIS2에서 매칭은 **여러 곳**에서 일어납니다:

| 함수 | 용도 | 현재 상태 | 문제점 |
|------|------|----------|--------|
| `matchStereo` | 현재 프레임 내 카메라 간 매칭 | ✅ LightGlue 사용 | 없음 |
| `matchMotionStereo` | 시간 순서 프레임 간 매칭 | ❌ BRISK Hamming | 양자화된 SuperPoint로 BRISK 매칭 |
| `matchToMap` | 3D 랜드마크와 2D 키포인트 매칭 | ❌ BRISK Hamming | 양자화된 SuperPoint로 BRISK 매칭 |
| `verifyRecognisedPlace` | Loop Closure 검증 | ❌ BRISK Hamming | 양자화된 SuperPoint로 BRISK 매칭 |

### 매칭 비중 (추정)

```
matchToMap (3D-2D tracking): ~60%
matchMotionStereo (temporal): ~30%
matchStereo (stereo init): ~10%  ← 현재 LightGlue 적용된 유일한 부분
```

**결론**: LightGlue가 전체 매칭의 약 10%에서만 사용되고 있음!

---

## 문제가 되는 코드 위치

### 1. matchMotionStereo (line 1904)
```cpp
const uint32_t dist = brisk::Hamming::PopcntofXORed(
    d0, desc1.data+kk*48, 3);  // ❌ 48바이트 BRISK 방식
```

### 2. matchToMapByThread (line 1638)
```cpp
const double dist = brisk::Hamming::PopcntofXORed(
    descriptorK,
    it->second.descriptors.data + d*48, 3);  // ❌ 48바이트 BRISK 방식
```

### 3. verifyRecognisedPlace (line 396)
```cpp
const uint32_t dist = brisk::Hamming::PopcntofXORed(
    ddata + 48 * k, oldDescripor, 3);  // ❌ 48바이트 BRISK 방식
```

---

## 왜 성능이 나빠지는가?

### 현재 상황:
1. SuperPoint로 float 디스크립터(256차원) 추출
2. float → uchar(48바이트)로 양자화 (정보 손실)
3. matchToMap, matchMotionStereo에서 **양자화된 48바이트 uchar**로 BRISK Hamming 매칭
4. 잘못된 매칭 또는 매칭 실패

### 문제의 근본 원인:
- BRISK 디스크립터: 48바이트 binary (384비트)
- SuperPoint 디스크립터: 256차원 float
- **현재 양자화**: `descriptors.convertTo(descriptors_uchar, CV_8U, 255.0)`
  - 256차원 float → 256바이트 uchar (단순 양자화)
  - 하지만 OKVIS는 48바이트를 기대함!

**⚠️ 치명적 버그**: 256바이트 디스크립터를 48바이트로 읽어서 사용 중!

---

## 추가 문제들

### 1. 디스크립터 크기 불일치 (치명적)
```cpp
// SuperPoint: 256 float = 1024 bytes → 256 uchar로 변환
// BRISK: 48 bytes
// OKVIS는 48 bytes를 기대하지만 256 bytes가 저장됨
memcpy(features.at(k + offset).data(),
       multiFrame.second->keypointDescriptor(im, k),
       48 * sizeof(uchar));  // 처음 48바이트만 사용!
```

### 2. L2 정규화 누락
SuperPoint 디스크립터는 일반적으로 L2 정규화가 필요하지만 현재는 누락됨.

### 3. SuperPoint ONNX 모델 출력 형식
- 키포인트: `[1, N, 2]` (x, y)
- 스코어: `[1, N]` (사용되지 않음)
- 디스크립터: `[1, N, 256]` (float)

---

## 해결 방안

### 옵션 1: 모든 매칭에서 LightGlue 사용 (권장)
- matchMotionStereo, matchToMap에도 LightGlue 적용
- 가장 정확하지만 구현 복잡도 높음

### 옵션 2: SuperPoint 비활성화, BRISK만 사용
- 현재 구현이 제대로 작동하지 않으므로 baseline으로 복귀
- `use_superpoint: false`, `use_lightglue: false`

### 옵션 3: 디스크립터 호환성 수정 (중간 복잡도)
- SuperPoint 디스크립터를 48바이트로 맞추기 (PCA 등)
- 또는 OKVIS의 디스크립터 크기를 256바이트로 변경

---

## 즉시 확인 필요한 사항

1. **디스크립터 저장 크기 확인**:
   ```cpp
   cv::Mat descriptors;  // 실제 크기 확인 필요
   ```

2. **BRISK 매칭에서 사용되는 디스크립터 크기**:
   - 48바이트를 가정하지만 SuperPoint는 256바이트 저장

3. **실제 매칭 개수 로그 확인**:
   - matchStereo, matchMotionStereo, matchToMap에서 매칭 개수

---

## 결론

**현재 상태**: SuperPoint 디스크립터가 BRISK 방식으로 매칭되어 심각한 성능 저하 발생

**주요 원인**:
1. LightGlue가 matchStereo에서만 사용됨 (전체의 ~10%)
2. 나머지 90% 매칭에서 양자화된 SuperPoint 디스크립터를 BRISK 방식으로 매칭
3. 디스크립터 크기 불일치 (256바이트 vs 48바이트)

**권장 조치**:
1. 당장 테스트: `use_superpoint: false`로 baseline 복귀
2. 장기적: 전체 매칭 파이프라인에 LightGlue 통합

