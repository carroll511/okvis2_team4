# 메모리 부족(OOM) 문제 해결 가이드

OKVIS2 실행 중 프로세스가 killed되는 현상은 주로 메모리 부족(OOM - Out Of Memory) 때문입니다.

## 문제 진단

### 1. 시스템 메모리 확인
```bash
# 호스트 시스템에서
free -h
```

### 2. Docker 메모리 제한 확인
```bash
# Docker 데몬의 메모리 제한 확인
docker info | grep -i memory

# 컨테이너 메모리 사용량 확인
docker stats okvis2_baseline_container
```

### 3. OOM Killer 로그 확인
```bash
# 시스템 로그에서 OOM 이벤트 확인
dmesg | grep -i "out of memory"
dmesg | grep -i "killed process"
journalctl -k | grep -i "out of memory"
```

## 해결 방법

### 방법 1: Docker Compose에서 메모리 제한 제거 (권장)

`network_mode: host`를 사용하는 경우, `deploy` 섹션이 작동하지 않을 수 있습니다.
대신 `docker run` 명령어를 사용하여 메모리 제한을 설정하거나 제거할 수 있습니다:

```bash
# 기존 컨테이너 중지
docker compose -f docker/docker-compose-baseline.yml down

# 메모리 제한 없이 직접 실행
docker run --rm -it \
    --gpus all \
    --network host \
    --privileged \
    --memory=0 \
    --memory-swap=0 \
    -v $(pwd):/workspace/okvis2:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev:rw \
    -v okvis2_baseline_external_modules:/workspace/external_modules \
    -v /home/junwan/1.Datasets/HILTI22:/workspace/datasets/HILTI22:ro \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -e ROS_DOMAIN_ID=0 \
    okvis2:latest \
    /bin/bash
```

### 방법 2: 스왑 공간 추가

스왑 공간을 추가하면 메모리 부족 시 디스크를 사용하여 프로세스를 계속 실행할 수 있습니다.

```bash
# 현재 스왑 확인
swapon --show
free -h

# 스왑 파일 생성 (예: 16GB)
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 영구적으로 활성화
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# 스왑 사용 우선순위 조정 (선택사항)
# /etc/sysctl.conf에 추가:
# vm.swappiness=10  # 0-100, 낮을수록 메모리 우선 사용
```

### 방법 3: Docker 데몬 메모리 제한 확인 및 제거

Docker Desktop이나 Docker Engine에 메모리 제한이 설정되어 있을 수 있습니다.

```bash
# Docker Desktop: Settings > Resources > Advanced > Memory
# 또는 docker-compose.yml에서 제거

# Docker Engine 설정 확인
cat /etc/docker/daemon.json
```

### 방법 4: 알고리즘 설정 최적화

OKVIS2 설정 파일에서 메모리 사용을 줄일 수 있는 옵션:

```yaml
# config/hilti_challenge_2022.yaml
estimator:
  # 최대 키프레임 수 제한
  max_keyframes: 100  # 기본값보다 낮게 설정
  
  # 루프 클로저 비활성화 (메모리 사용량 감소)
  do_loop_closures: false  # true -> false
  
  # 최종 BA 비활성화 (메모리 사용량 감소)
  do_final_ba: false  # true -> false
```

### 방법 5: 배치 처리 또는 청크 단위 처리

대용량 데이터셋의 경우, 데이터를 청크로 나누어 처리하는 것을 고려하세요.

## 모니터링

실행 중 메모리 사용량을 모니터링:

```bash
# 별도 터미널에서
watch -n 1 'docker stats okvis2_baseline_container --no-stream'

# 또는 호스트 시스템 전체
watch -n 1 'free -h'
```

## 추가 팁

1. **다른 프로세스 종료**: 불필요한 프로세스를 종료하여 메모리 확보
2. **컨테이너 재시작**: 주기적으로 컨테이너를 재시작하여 메모리 누수 방지
3. **데이터셋 전처리**: 가능하면 데이터셋을 작은 단위로 나누어 처리
4. **하드웨어 업그레이드**: RAM 추가가 가장 확실한 해결책

## 참고

- Docker 메모리 제한: https://docs.docker.com/config/containers/resource_constraints/
- Linux OOM Killer: https://www.kernel.org/doc/Documentation/admin-guide/oom-killer.rst

