# OKVIS2 Docker 환경

이 디렉토리에는 OKVIS2를 Docker 환경에서 빌드하고 실행하기 위한 모든 파일이 포함되어 있습니다.

## 파일 구조

- `Dockerfile` - Docker 이미지 빌드 정의
- `docker-compose.yml` - Docker Compose 설정 파일
- `.dockerignore` - Docker 빌드 시 제외할 파일 목록
- `docker_build.sh` - 컨테이너 내부에서 OKVIS2를 빌드하는 헬퍼 스크립트

## 문서

- **[DOCKER_SETUP.md](DOCKER_SETUP.md)** - Docker 환경 구성 및 이미지 빌드 가이드
- **[BUILD_AND_RUN.md](BUILD_AND_RUN.md)** - OKVIS2 빌드 및 실행 가이드
- **[SUPERVINS_MODEL_USAGE.md](SUPERVINS_MODEL_USAGE.md)** - SuperVINS ONNX 모델 사용 가이드

## 빠른 시작

### 1. Docker 이미지 빌드

```bash
cd docker
docker compose build
```

### 2. 컨테이너 실행

```bash
cd docker
docker compose up -d
docker compose exec okvis2 bash
```

### 3. OKVIS2 빌드 및 실행

컨테이너 내부에서:

```bash
cd /workspace/okvis2
./docker/docker_build.sh
```

자세한 내용은 [BUILD_AND_RUN.md](BUILD_AND_RUN.md)를 참고하세요.

## 주의사항

- Docker Compose 명령어는 `docker compose` (하이픈 없이)를 사용합니다
- GPU 지원을 위해 NVIDIA Docker가 필요합니다
- 컨테이너 내부에서 빌드할 때는 `/workspace/okvis2` 경로를 사용합니다
- 데이터셋은 `/workspace/datasets/HILTI22` 경로로 마운트됩니다 (호스트: `/home/junwan/1.Datasets/HILTI22`)
