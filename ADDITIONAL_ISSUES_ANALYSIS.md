# SuperPoint/LightGlue 성능 저하 - 추가 문제 분석

## 🔴 발견된 심각한 문제들

### 1. **LightGlue 키포인트 정규화 누락** (매우 심각)

**문제 위치**: `LightGlueMatcher.cpp:276-278`

```cpp
// Note: SuperVINS typically normalizes keypoints, but we'll use original coordinates
// If needed, we can add image dimensions and normalize
// For now, use keypoints as-is (they may already be normalized from extractor)
```

**문제점**:
- LightGlue는 **정규화된 키포인트**를 기대함 (SuperVINS 스타일)
- 현재는 원본 픽셀 좌표를 그대로 사용
- LightGlue 모델이 학습된 형식과 다름 → 매칭 성능 대폭 저하

**SuperVINS 방식**:
```python
# SuperVINS는 키포인트를 정규화해서 LightGlue에 전달
kpts0_norm = normalize_keypoints(kpts0, h0, w0)  # [-1, 1] 범위로 정규화
kpts1_norm = normalize_keypoints(kpts1, h1, w1)
```

**영향**: ⚠️ 매우 심각 - 매칭 정확도 대폭 감소

---

### 2. **SuperPoint 키포인트-디스크립터 인덱스 불일치** (심각)

**문제 위치**: `SuperPointExtractor_SuperVINS.cpp:228-257`

```cpp
for (const auto& kpt : kpts) {
    // Scale back to original image coordinates
    cv::Point2f pt = cv::Point2f((kpt.x + 0.5f) / scale_ - 0.5f, 
                                  (kpt.y + 0.5f) / scale_ - 0.5f);
    
    // Check bounds
    if (pt.x < 0 || pt.x >= image.cols || pt.y < 0 || pt.y >= image.rows) {
      continue;  // ❌ 키포인트를 스킵하지만 디스크립터는 그대로 사용
    }
    keypoints.push_back(kp);
}

// 디스크립터는 원본 kpts 인덱스로 접근
for (size_t i = 0; i < keypoints.size() && i < kpts.size(); ++i) {
    float* desc_ptr = desc_data + i * 256;  // ❌ 인덱스 불일치!
}
```

**문제점**:
- Bounds check로 일부 키포인트가 필터링됨
- 하지만 디스크립터는 원본 인덱스로 접근
- 키포인트와 디스크립터가 매칭되지 않음

**영향**: ⚠️ 심각 - 잘못된 디스크립터로 매칭 시도

---

### 3. **LightGlue Confidence 정보 손실** (중간)

**문제 위치**: `LightGlueMatcher.cpp:290`

```cpp
m.confidence = 1.0f;  // ❌ SuperVINS doesn't return confidence in match_featurepoints
```

**문제점**:
- `post_process()`에서 `mscores`를 읽어오지만 사용하지 않음
- 모든 매칭에 confidence = 1.0으로 설정
- Threshold 필터링이 제대로 작동하지 않음

**실제 코드**:
```cpp
// post_process()에서는 mscores를 읽음
float* mscores = (float*)outputtensors_[1].GetTensorMutableData<void>();
if (mscores[i] > match_threshold_) {  // 여기서는 threshold 사용
    good_matches.emplace_back(...);
}
// 하지만 match()에서는 confidence를 1.0으로 설정
m.confidence = 1.0f;  // ❌ mscores 정보 손실
```

**영향**: ⚠️ 중간 - Threshold 필터링이 제대로 작동하지 않음

---

### 4. **이미지 크기 정보 누락** (중간)

**문제 위치**: `LightGlueMatcher.cpp:match()`

**문제점**:
- LightGlue에 키포인트를 정규화하려면 이미지 크기(h, w)가 필요
- 하지만 `match()` 함수에 이미지 크기 정보가 전달되지 않음
- `pre_process()` 함수는 있지만 호출되지 않음

**영향**: ⚠️ 중간 - 키포인트 정규화 불가

---

### 5. **SuperPoint 키포인트 스케일링 공식 문제** (의심)

**문제 위치**: `SuperPointExtractor_SuperVINS.cpp:230`

```cpp
cv::Point2f pt = cv::Point2f((kpt.x + 0.5f) / scale_ - 0.5f, 
                              (kpt.y + 0.5f) / scale_ - 0.5f);
```

**문제점**:
- SuperVINS의 정확한 스케일링 공식과 일치하는지 확인 필요
- `+0.5f`와 `-0.5f`의 의미가 명확하지 않음

**영향**: ⚠️ 낮음 - 하지만 확인 필요

---

### 6. **디스크립터 차원 확인 필요** (확인 필요)

**문제 위치**: `LightGlueMatcher.cpp:154`

```cpp
int desc_dim = (extractor_type_ == SUPERPOINT) ? SUPERPOINT_SIZE : DISK_SIZE;
```

**확인 필요**:
- `SUPERPOINT_SIZE`가 256인지 확인
- 디스크립터가 올바른 형식인지 확인

---

## 📊 문제 우선순위

| 순위 | 문제 | 심각도 | 예상 영향 |
|------|------|--------|----------|
| 1 | LightGlue 키포인트 정규화 누락 | 🔴 매우 심각 | 매칭 정확도 50-70% 감소 |
| 2 | 키포인트-디스크립터 인덱스 불일치 | 🔴 심각 | 잘못된 매칭 20-30% |
| 3 | Confidence 정보 손실 | 🟡 중간 | Threshold 필터링 실패 |
| 4 | 이미지 크기 정보 누락 | 🟡 중간 | 정규화 불가 |
| 5 | 스케일링 공식 | 🟢 낮음 | 미세한 오차 |

---

## 🔧 해결 방안

### 즉시 수정 필요:

1. **LightGlue 키포인트 정규화 추가**
   - `match()` 함수에 이미지 크기 파라미터 추가
   - `NormalizeKeypoints()` 호출

2. **키포인트-디스크립터 인덱스 수정**
   - 필터링된 키포인트에 대응하는 디스크립터만 사용
   - 인덱스 매핑 테이블 생성

3. **Confidence 정보 전달**
   - `post_process()`에서 confidence를 반환
   - `match()`에서 실제 confidence 사용

---

## 🎯 예상 성능 개선

- **키포인트 정규화 수정**: 30-50% 성능 향상 예상
- **인덱스 불일치 수정**: 15-25% 성능 향상 예상
- **Confidence 수정**: 10-15% 성능 향상 예상
- **전체 개선**: 50-80% 성능 향상 가능

---

## 📝 추가 확인 사항

1. SuperVINS의 정확한 키포인트 정규화 공식 확인
2. SuperPoint 출력 형식 확인 (키포인트, 스코어, 디스크립터 순서)
3. LightGlue 입력 형식 확인 (정규화 범위, 차원 등)
4. 실제 실행 시 로그 확인 (매칭 개수, confidence 분포 등)

