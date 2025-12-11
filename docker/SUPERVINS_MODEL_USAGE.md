# SuperVINS ONNX 모델 사용 가이드

SuperVINS에서 제공하는 ONNX 모델 파일들을 OKVIS2에서 그대로 사용할 수 있습니다.

## 지원하는 모델

1. **superpoint.onnx** - SuperPoint feature extractor
2. **superpoint_lightglue_fused_cpu.onnx** - SuperPoint + LightGlue fused model (CPU)
3. **superpoint_lightglue_fused_gpu.onnx** - SuperPoint + LightGlue fused model (GPU) (있는 경우)

## 모델 다운로드

### 방법 1: SuperVINS 저장소에서 직접 다운로드

```bash
# SuperVINS 저장소 클론
git clone https://github.com/luohongk/SuperVINS.git
cd SuperVINS

# 모델 파일 확인
ls -la supervins_estimator/weights_dpl/
# superpoint.onnx
# superpoint_lightglue_fused_cpu.onnx

# OKVIS2 프로젝트로 복사
mkdir -p /workspace/models
cp supervins_estimator/weights_dpl/*.onnx /workspace/models/
```

### 방법 2: SuperVINS의 download_data.sh 사용

```bash
cd SuperVINS
chmod +x download_data.sh
./download_data.sh
# 이 스크립트가 모델 파일을 다운로드할 수 있습니다
```

## 설정 방법

### Separate 모델 사용 (SuperPoint + LightGlue)

```yaml
frontend_parameters:
  # SuperVINS에서 다운로드한 모델 경로
  extractor_weight_path: "/workspace/models/superpoint.onnx"
  matcher_weight_path: "/workspace/models/superpoint_lightglue_fused_cpu.onnx"
  
  use_superpoint: true
  use_lightglue: true
  use_fused_model: false  # Separate models
  
  use_gpu_for_nn: false  # CPU fused model 사용 시
  # use_gpu_for_nn: true  # GPU 사용 시
  
  superpoint_keypoint_threshold: 0.015
  superpoint_max_keypoints: 2048
  lightglue_match_threshold: 0.5  # SuperVINS 기본값
```

### Fused 모델 사용

```yaml
frontend_parameters:
  # Fused 모델 사용 (SuperPoint + LightGlue 통합)
  extractor_weight_path: "/workspace/models/superpoint_lightglue_fused_cpu.onnx"
  matcher_weight_path: ""  # Fused 모델 사용 시 비워둠
  
  use_superpoint: true
  use_lightglue: true
  use_fused_model: true  # Fused model 사용
  
  use_gpu_for_nn: false  # CPU fused model
  # use_gpu_for_nn: true  # GPU fused model (있는 경우)
  
  superpoint_keypoint_threshold: 0.015
  superpoint_max_keypoints: 2048
  lightglue_match_threshold: 0.5
```

## 모델 형식

### SuperPoint 모델 출력

SuperVINS의 SuperPoint 모델은 다음 형식으로 출력합니다:
- **tensor[0]**: keypoints (int64_t) [N, 2] - 키포인트 좌표
- **tensor[1]**: scores (float) [N] - 키포인트 점수
- **tensor[2]**: descriptors (float) [N, 256] - 디스크립터

### LightGlue 모델 입력/출력

SuperVINS의 LightGlue 모델은 다음 형식을 사용합니다:

**입력:**
- **kpts0**: keypoints (float) [1, N, 2]
- **kpts1**: keypoints (float) [1, M, 2]
- **desc0**: descriptors (float) [1, N, 256] (SuperPoint) 또는 [1, N, 128] (DISK)
- **desc1**: descriptors (float) [1, M, 256] (SuperPoint) 또는 [1, M, 128] (DISK)

**출력:**
- **matches**: match indices (int64_t) [N, 2] - 매칭 인덱스 쌍 [idx0, idx1]
- **mscores**: match scores (float) [N] - 매칭 점수

## 실행 예제

```bash
# 컨테이너 내부에서
cd /workspace/okvis2

# 빌드
./docker/docker_build.sh

# 환경 설정
source /opt/ros/humble/setup.bash
source /workspace/okvis2/build/install/share/okvis/local_setup.bash

# 실행 (SuperVINS 모델 사용)
ros2 launch okvis okvis_node_synchronous.launch.xml \
    config_filename:=/workspace/okvis2/config/euroc_supervins_style.yaml \
    path:=/workspace/datasets/HILTI22/exp02_construction_multilevel \
    topic_prefix:=/alphasense
```

## 주의사항

1. **모델 경로**: 절대 경로 또는 설정 파일 기준 상대 경로 사용
2. **GPU/CPU**: `use_gpu_for_nn` 설정에 따라 적절한 모델 사용
   - CPU 모델: `superpoint_lightglue_fused_cpu.onnx`
   - GPU 모델: `superpoint_lightglue_fused_gpu.onnx` (있는 경우)
3. **Match Threshold**: SuperVINS 기본값은 0.5입니다 (기존 0.2에서 변경됨)
4. **Extractor Type**: 현재는 SuperPoint (0)만 지원, DISK (1)는 추후 지원 예정

## 문제 해결

### 모델 로드 실패

```bash
# 모델 파일 확인
ls -lh /workspace/models/*.onnx

# 파일 권한 확인
chmod 644 /workspace/models/*.onnx
```

### ONNX Runtime 오류

```bash
# ONNX Runtime 버전 확인
python3 -c "import onnxruntime; print(onnxruntime.__version__)"

# 필요한 경우 업데이트
pip install --upgrade onnxruntime-gpu  # GPU 사용 시
# 또는
pip install --upgrade onnxruntime  # CPU 사용 시
```

### GPU 메모리 부족

```yaml
# CPU 모델 사용
use_gpu_for_nn: false
extractor_weight_path: "/workspace/models/superpoint_lightglue_fused_cpu.onnx"
```

## 참고

- SuperVINS GitHub: https://github.com/luohongk/SuperVINS
- SuperVINS 논문: IEEE Sensors Journal (2025)
- ONNX Runtime 문서: https://onnxruntime.ai/docs/

