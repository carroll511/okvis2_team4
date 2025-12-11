# OKVIS2 빌드 및 실행 가이드

이 문서는 Docker 컨테이너 내부에서 OKVIS2를 빌드하고 실행하는 방법을 설명합니다.

## 컨테이너 접속

### Docker Compose 사용

**`docker` 디렉토리에서 실행:**
```bash
cd docker
docker compose exec okvis2 bash
```

**프로젝트 루트에서 실행:**
```bash
docker compose -f docker/docker-compose.yml exec okvis2 bash
```

### 직접 Docker 명령어 사용

```bash
docker exec -it okvis2_container bash
```

## OKVIS2 빌드

### 방법 1: 빌드 스크립트 사용 (권장)

```bash
cd /workspace/okvis2
./docker/docker_build.sh
```

### 빌드 후 ROS2 환경 설정

빌드가 완료되면 ROS2 패키지를 사용하기 위해 워크스페이스를 소스해야 합니다:

```bash
# ROS2 기본 환경 소스
source /opt/ros/humble/setup.bash

# OKVIS2 워크스페이스 소스 (install 디렉토리 - 필수!)
# ROS2 ament 패키지는 install/share/<package_name>/local_setup.bash에 설치됩니다
source /workspace/okvis2/build/install/share/okvis/local_setup.bash
```

**참고:** 
- `docker_build.sh` 스크립트는 자동으로 `make install`을 실행하여 `build/install/` 디렉토리에 패키지를 설치합니다.
- `install/share/okvis/local_setup.bash`를 소스하면 OKVIS2 ROS2 패키지(launch 파일 포함)가 ROS2 환경에 등록됩니다.
- 이 명령어는 매 터미널 세션마다 실행해야 합니다.
- `.bashrc`에 추가하여 자동으로 소스되도록 할 수 있습니다.

### 방법 2: 수동 빌드

```bash
# ROS2 환경 소스
source /opt/ros/humble/setup.bash

# 빌드 디렉토리 생성 및 이동
cd /workspace/okvis2
mkdir -p build && cd build

# CMake 설정
cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_ROS2=ON \
    -DUSE_NN=ON \
    -DUSE_GPU=ON \
    -DHAVE_LIBREALSENSE=OFF \
    -DTorch_DIR=/opt/libtorch/share/cmake/Torch \
    ..

# 빌드 (시간이 걸릴 수 있습니다)
make -j$(nproc)
```

### 방법 3: colcon을 사용한 ROS2 워크스페이스 빌드

```bash
# 워크스페이스 생성
mkdir -p ~/okvis_ws/src
cd ~/okvis_ws/src
ln -s /workspace/okvis2 .

# 의존성 설치
cd ~/okvis_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --cmake-args -DBUILD_ROS2=ON -DUSE_NN=ON -DUSE_GPU=ON

# 환경 소스 (colcon build를 사용한 경우)
source install/setup.bash

# 참고: make install을 사용한 경우 (docker_build.sh 사용 시):
# source /workspace/okvis2/build/install/share/okvis/local_setup.bash
```

## 빌드 확인

빌드가 완료되면 실행 파일이 생성됩니다:

```bash
# 빌드된 실행 파일 확인
ls -lh /workspace/okvis2/build/okvis_app_*

# 또는 ROS2 노드 확인
ls -lh /workspace/okvis2/build/okvis_node_*
```

## OKVIS2 실행 방법

**실행 전 필수 사항:**
모든 ROS2 노드를 실행하기 전에 다음 명령어로 환경을 설정해야 합니다:

```bash
# ROS2 기본 환경 소스
source /opt/ros/humble/setup.bash

# OKVIS2 워크스페이스 소스
# docker_build.sh를 사용한 경우 (build/install에 설치됨):
source /workspace/okvis2/build/install/share/okvis/local_setup.bash

# 또는 수동 빌드로 /usr/local에 설치한 경우:
source /usr/local/share/okvis/local_setup.bash
```

**중요:** 
- `docker_build.sh` 스크립트는 자동으로 `make install`을 실행하며, `CMAKE_INSTALL_PREFIX`를 `build/install`로 설정합니다.
- 만약 수동으로 빌드했다면 다음 명령어로 설치해야 합니다:
  ```bash
  cd /workspace/okvis2/build
  make install
  ```
- 수동 빌드 시 기본 설치 경로는 `/usr/local`이므로, `/usr/local/share/okvis/local_setup.bash`를 소스해야 합니다.

OKVIS2는 다음 4가지 방식으로 실행할 수 있습니다:

### 1. EuRoC 형식 데이터셋 (동기식 처리)

EuRoC MAV 데이터셋 형식을 지원합니다.

**데이터셋 구조:**
```
dataset_folder/
├── mav0/
│   ├── cam0/
│   │   ├── data/
│   │   └── data.csv
│   ├── cam1/
│   │   ├── data/
│   │   └── data.csv
│   └── imu0/
│       └── data.csv
```

**ROS2 노드로 실행:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch okvis okvis_node_synchronous.launch.xml \
    config_filename:=/workspace/okvis2/config/euroc.yaml \
    path:=/path/to/euroc/dataset/MH_01_easy/mav0
```

**직접 실행 (ROS2 없이):**
```bash
cd /workspace/okvis2/build
./okvis_app_synchronous \
    /workspace/okvis2/config/euroc.yaml \
    /path/to/euroc/dataset/MH_01_easy/mav0
```

### 2. ROS2 Bag 파일 (동기식 처리)

ROS2 bag 파일 (`.db3`, `.mcap` 형식)을 지원합니다.

**Bag 파일 구조:**
```
bag_folder/
├── metadata.yaml
└── bag_file.db3  (또는 .mcap)
```

**필수 토픽:**
- `/okvis/imu0` (sensor_msgs/Imu)
- `/okvis/cam0/image_raw` (sensor_msgs/Image)
- `/okvis/cam1/image_raw` (sensor_msgs/Image) - 스테레오인 경우

**데이터셋 마운트:**
Docker 컨테이너는 호스트의 `/home/junwan/1.Datasets/HILTI22` 디렉토리를 `/workspace/datasets/HILTI22`로 마운트합니다.

**실행 명령어:**
```bash
# ROS2 기본 환경 소스
source /opt/ros/humble/setup.bash

# OKVIS2 워크스페이스 소스 (필수!)
# ROS2 ament 패키지는 install/share/<package_name>/local_setup.bash에 설치됩니다
source /workspace/okvis2/build/install/share/okvis/local_setup.bash

# Launch 파일 실행
ros2 launch okvis okvis_node_synchronous.launch.xml \
    config_filename:=/workspace/okvis2/config/euroc.yaml \
    path:=/workspace/datasets/HILTI22/path/to/bag/folder \
    topic_prefix:=/alphasense \
    output_path:=/workspace/okvis2/output  # Optional: trajectory 파일 출력 경로 (기본값: path 또는 /workspace/okvis2/output)
# 참고: topic_prefix는 HILTI22 데이터셋의 경우 /alphasense를 사용 (기본값: /okvis)
```

**중요:** 
- `ros2 launch` 명령어를 실행하기 전에 반드시 `install/share/okvis/local_setup.bash`를 소스해야 합니다.
- 빌드 후 `make install`이 실행되었는지 확인하세요. 그렇지 않으면 "Package 'okvis' not found" 오류가 발생합니다.
- **Trajectory 파일 출력**: 데이터셋 디렉토리가 읽기 전용(`:ro`)으로 마운트된 경우, trajectory 파일은 자동으로 `/workspace/okvis2/output/` 디렉토리에 저장됩니다. `output_path` 파라미터로 다른 경로를 지정할 수 있습니다.
- **topic_prefix 파라미터**: ROS2 bag 파일의 토픽 이름 prefix를 지정합니다. HILTI22 데이터셋은 `/alphasense`를 사용합니다. bag 파일의 실제 토픽 이름을 확인하려면 `ros2 bag info <bag_path>`를 실행하세요.

**예제 (HILTI22 데이터셋):**
```bash
# HILTI22 데이터셋의 특정 시퀀스 실행
# ⚠️ 중요: HILTI22 데이터셋은 topic_prefix:=/alphasense를 반드시 지정해야 합니다!
ros2 launch okvis okvis_node_synchronous.launch.xml \
    config_filename:=/workspace/okvis2/config/hilti_challenge_2022.yaml \
    path:=/workspace/datasets/HILTI22/exp02_construction_multilevel \
    topic_prefix:=/alphasense \
    output_path:=/workspace/okvis2/output  # Trajectory 파일 출력 경로 (선택사항)
```

**topic_prefix 파라미터 설명:**
- **기본값**: `/okvis` (EuRoC 등 표준 데이터셋용)
- **HILTI22 데이터셋**: `/alphasense` (필수!)
- **다른 데이터셋**: bag 파일의 실제 토픽 이름에 맞게 설정
  - 예: bag에 `/my_robot/imu`가 있으면 `topic_prefix:=/my_robot` 사용

**Trajectory 파일 위치:**
- 데이터셋 디렉토리가 읽기 전용인 경우: `/workspace/okvis2/output/okvis2-*-live_trajectory.csv`
- 데이터셋 디렉토리가 쓰기 가능한 경우: `path/okvis2-*-live_trajectory.csv`
- `output_path`를 지정한 경우: `output_path/okvis2-*-live_trajectory.csv`

**참고:** 
- `okvis_node_synchronous`는 `path` 디렉토리에 `metadata.yaml` 파일이 있으면 자동으로 ROS2 bag으로 인식합니다.
- 다른 데이터셋 디렉토리를 사용하려면 `docker-compose.yml`의 `volumes` 섹션에 추가 마운트를 설정하세요.

### 3. RealSense 카메라 (라이브 실행)

RealSense D435i 또는 D455 카메라를 직접 사용합니다.

**전제 조건:**
- RealSense 카메라가 연결되어 있어야 함
- 컨테이너가 `--privileged` 모드로 실행되어야 함

**실행 명령어:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch okvis okvis_node_realsense.launch.xml \
    config_filename:=/workspace/okvis2/config/realsense_D455.yaml
```

**설정 파일:**
- `config/realsense_D435i.yaml` - D435i용
- `config/realsense_D455.yaml` - D455용

### 4. ROS2 토픽 구독 (라이브 실행)

다른 노드가 발행하는 ROS2 토픽을 구독하여 실행합니다.

**필수 토픽:**
- `/okvis/imu0` (sensor_msgs/Imu)
- `/okvis/cam0/image_raw` (sensor_msgs/Image)
- `/okvis/cam1/image_raw` (sensor_msgs/Image) - 스테레오인 경우

**실행 방법:**

1. **RealSense Publisher 실행 (다른 터미널):**
```bash
source /opt/ros/humble/setup.bash
ros2 launch okvis okvis_node_realsense_publisher.launch.xml \
    config_filename:=/workspace/okvis2/config/realsense_D455.yaml
```

2. **Subscriber 실행:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch okvis okvis_node_subscriber.launch.xml \
    config_filename:=/workspace/okvis2/config/realsense_D455.yaml
```

**토픽 확인:**
```bash
# 발행 중인 토픽 확인
ros2 topic list

# 특정 토픽 확인
ros2 topic echo /okvis/imu0
ros2 topic hz /okvis/cam0/image_raw
```

## 추가 옵션

### RVIZ 비활성화
```bash
ros2 launch okvis okvis_node_synchronous.launch.xml \
    config_filename:=/workspace/okvis2/config/euroc.yaml \
    path:=/path/to/dataset \
    rviz:=false
```

### RPG 데이터셋 형식 사용
```bash
ros2 launch okvis okvis_node_synchronous.launch.xml \
    config_filename:=/workspace/okvis2/config/euroc.yaml \
    path:=/path/to/rpg/dataset \
    rpg:=true
```

### RGB 이미지 사용
```bash
ros2 launch okvis okvis_node_synchronous.launch.xml \
    config_filename:=/workspace/okvis2/config/euroc.yaml \
    path:=/path/to/dataset \
    rgb:=true
```

## 출력 파일

동기식 처리 시 다음 파일들이 생성됩니다:

- `okvis2-slam-live_trajectory.csv` - 실시간 궤적
- `okvis2-slam-final_trajectory.csv` - 최종 최적화된 궤적
- `okvis2-slam-final_map.csv` - 맵 데이터

파일은 `path` 파라미터로 지정한 디렉토리에 저장됩니다.

## SuperPoint & LightGlue 사용

### 설치 확인

```bash
python3 -c "import kornia; from kornia.feature import SuperPoint, LightGlue; print('Available!')"
```

### 설치 (필요시)

```bash
# pip로 설치
pip3 install superpoint lightglue

# 또는 소스에서 설치
cd /workspace/external_modules
git clone https://github.com/magicleap/SuperPointPretrainedNetwork.git
cd SuperPointPretrainedNetwork
pip3 install -e .

cd /workspace/external_modules
git clone https://github.com/cvg/LightGlue.git
cd LightGlue
pip3 install -e .
```

### Python에서 사용 예시

```python
import torch
import kornia as K
from kornia.feature import SuperPoint, LightGlue

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# SuperPoint 초기화
superpoint = SuperPoint(max_num_keypoints=2048).to(device)
superpoint.eval()

# LightGlue 초기화
lightglue = LightGlue(features='superpoint').to(device)
lightglue.eval()
```

## 유용한 명령어

### 환경 확인

```bash
# GPU 확인
nvidia-smi

# PyTorch CUDA 지원 확인
python3 -c "import torch; print(torch.cuda.is_available())"

# ROS2 환경 확인
echo $ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
```

### 문제 해결

**데이터셋 경로 문제:**
```bash
# 절대 경로 사용 권장
path:=/data/euroc/MH_01_easy/mav0
```

**RealSense 카메라 인식 안 됨:**
```bash
# 호스트에서 카메라 확인
lsusb | grep Intel
```

**ROS2 토픽이 보이지 않음:**
```bash
# ROS2 도메인 ID 확인 (기본값: 0)
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0
```

## 실행 방식 요약

| 방식 | Launch 파일 | 데이터 소스 | 실시간 |
|------|------------|------------|--------|
| EuRoC 데이터셋 | `okvis_node_synchronous` | 파일 시스템 | ❌ |
| ROS2 Bag | `okvis_node_synchronous` | Bag 파일 | ❌ |
| RealSense 카메라 | `okvis_node_realsense` | 하드웨어 | ✅ |
| 토픽 구독 | `okvis_node_subscriber` | ROS2 토픽 | ✅ |

## 참고 자료

- [OKVIS2 README](../README.md)
- [설정 파일 가이드](../config/)
- [EuRoC 데이터셋](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

