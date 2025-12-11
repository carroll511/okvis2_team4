# Docker 환경 구성 및 빌드 가이드

이 문서는 OKVIS2를 위한 Docker 환경을 구성하고 이미지를 빌드하는 방법을 설명합니다.

## 요구사항

- Docker (20.10 이상)
- Docker Compose (1.29 이상) 또는 `docker compose` (V2)
- NVIDIA Docker (GPU 지원을 위해)
- NVIDIA GPU 드라이버 (CUDA 12.1 호환)

## 파일 구조

이 디렉토리에는 다음 파일들이 포함되어 있습니다:

- `Dockerfile` - Docker 이미지 빌드 정의
- `docker-compose.yml` - Docker Compose 설정 파일
- `.dockerignore` - Docker 빌드 시 제외할 파일 목록
- `docker_build.sh` - 컨테이너 내부에서 OKVIS2를 빌드하는 헬퍼 스크립트

## Docker 이미지 빌드

### 방법 1: Docker Compose 사용 (권장)

**`docker` 디렉토리에서 실행:**
```bash
cd docker
docker compose build
```

**프로젝트 루트에서 실행:**
```bash
docker compose -f docker/docker-compose.yml build
```

### 방법 2: 직접 Docker 명령어 사용

```bash
docker build -t okvis2:latest -f docker/Dockerfile .
```

### 빌드 시간

빌드는 시간이 걸릴 수 있습니다 (10-30분). 백그라운드로 실행하려면:

```bash
docker compose build > build.log 2>&1 &
tail -f build.log
```

## 컨테이너 실행

### Docker Compose 사용

**`docker` 디렉토리에서 실행:**
```bash
cd docker
docker compose up -d
docker compose exec okvis2 bash
```

**프로젝트 루트에서 실행:**
```bash
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec okvis2 bash
```

### 직접 Docker 명령어 사용

```bash
docker run --rm -it \
    --gpus all \
    --network host \
    --privileged \
    -v $(pwd):/workspace/okvis2 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    okvis2:latest
```

## 포함된 주요 구성 요소

### 시스템 패키지
- ROS2 Humble (Ubuntu 22.04 호환)
- CUDA 12.1 개발 도구
- OpenCV 4.x
- Eigen3
- Ceres Solver (서브모듈)
- SuiteSparse
- Boost
- Google Glog/Gflags

### 딥러닝 프레임워크
- PyTorch (Python) - CUDA 12.1 지원
- LibTorch (C++) - CUDA 12.1 지원
- torchvision, torchaudio

### 추가 라이브러리
- PIL/Pillow
- NumPy, SciPy
- scikit-image
- einops, timm (FlowFormer/MAC-VO용)
- kornia, kornia-moons (SuperPoint/LightGlue용)
- h5py, pyyaml, pycolmap, imageio

### 외부 모듈 통합 지원
- FlowFormer
- MAC-VO
- SuperPoint & LightGlue

## 환경 변수

- `Torch_DIR`: LibTorch 경로 (`/opt/libtorch`)
- `CUDA_HOME`: CUDA 설치 경로 (`/usr/local/cuda`)
- `ROS_DISTRO`: ROS2 배포판 (`humble`)
- `ROS_DOMAIN_ID`: ROS2 도메인 ID (기본값: 0)

## 문제 해결

### GPU가 인식되지 않는 경우

```bash
# NVIDIA Docker가 제대로 설치되었는지 확인
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

### RealSense 카메라 접근 문제

컨테이너를 `--privileged` 모드로 실행해야 합니다. `docker-compose.yml`에서 이미 설정되어 있습니다.

### LibTorch 경로 문제

```bash
export Torch_DIR=/opt/libtorch
export LD_LIBRARY_PATH=/opt/libtorch/lib:${LD_LIBRARY_PATH}
```

### ROS2 통신 문제

`network_mode: host`를 사용하거나 동일한 `ROS_DOMAIN_ID`를 설정하세요.

### CUDA 이미지를 찾을 수 없는 경우

Dockerfile의 베이스 이미지를 변경할 수 있습니다:

```dockerfile
# Ubuntu 22.04 사용 (현재 설정)
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

# 또는 더 최신 CUDA 버전 사용
FROM nvidia/cuda:12.4.0-devel-ubuntu22.04
```

### 빌드 중 메모리 부족

Docker의 메모리 제한을 늘리거나, 빌드 시 병렬 작업 수를 줄이세요:

```bash
# docker_build.sh에서 -j$(nproc) 대신 -j2 사용
make -j2
```

### 컨테이너 관리 명령어

```bash
# 컨테이너 상태 확인
docker compose ps

# 컨테이너 로그 확인
docker compose logs okvis2

# 컨테이너 중지
docker compose stop

# 컨테이너 재시작
docker compose restart

# 컨테이너 제거 (데이터는 유지됨)
docker compose down
```

## 참고 자료

- [OKVIS2 README](../README.md)
- [ROS2 Humble 문서](https://docs.ros.org/en/humble/)
- [PyTorch 설치 가이드](https://pytorch.org/get-started/locally/)
- [NVIDIA Docker 문서](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)

