# econ_image_mosic

4개의 카메라 이미지를 하나의 모자이크 이미지로 결합하는 ROS2 패키지입니다. 메모리 최적화와 다양한 하드웨어 가속을 지원하여 고성능 이미지 처리를 제공합니다.

## 주요 특징

- **메모리 최적화**: Iceoryx 메모리풀 부족 문제 해결
- **다중 하드웨어 가속**: CUDA GPU, PVI/NVENC 지원
- **압축 전송**: JPEG 압축으로 메모리 사용량 크게 감소
- **QoS 최적화**: BEST_EFFORT QoS로 호환성 문제 해결
- **실시간 처리**: 15-30Hz 실시간 모자이크 생성
- **자동 리소스 관리**: 가비지 컬렉션과 메모리 정리

## 입력 토픽

- `/dev/video0/image_raw` (sensor_msgs/Image)
- `/dev/video1/image_raw` (sensor_msgs/Image)  
- `/dev/video2/image_raw` (sensor_msgs/Image)
- `/dev/video3/image_raw` (sensor_msgs/Image)

## 출력 토픽

- `/camera/mosaic/image_raw/compressed` (sensor_msgs/CompressedImage) - 압축 모드 (기본값)
- `/camera/mosaic/image_raw` (sensor_msgs/Image) - 비압축 모드

## 모자이크 레이아웃

```
+----------+----------+
| Camera 0 | Camera 1 |
| (좌상단)  | (우상단)  |
+----------+----------+
| Camera 2 | Camera 3 |
| (좌하단)  | (우하단)  |
+----------+----------+
```

## 시스템 요구사항

### 필수 의존성
- ROS2 (Humble 권장)
- OpenCV 4.x
- cv_bridge
- sensor_msgs
- rclpy

### 권장 하드웨어
- NVIDIA Jetson 시리즈 (PVI/NVENC 가속)
- CUDA 지원 GPU (CUDA 가속)
- 최소 4GB RAM

## 설치 및 설정

### 1. 메모리 설정 최적화

먼저 Iceoryx 메모리풀 설정을 최적화해야 합니다:

```bash
# CycloneDDS 설정 확인
export CYCLONEDX_URI=/home/hycon/cyclonedds.xml

# Iceoryx 설정 파일 생성됨: /tmp/iceoryx_config.toml
```

### 2. 패키지 빌드

```bash
# 워크스페이스로 이동
cd ~/econ_gstreamer_ws

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --packages-select econ_image_mosic --symlink-install

# 환경 설정
source install/setup.bash
```

## 실행 방법

### 1. 기본 실행 (압축 모드, 최적화된 설정)
```bash
ros2 launch econ_image_mosic image_mosaic.launch.py
```

### 2. 고화질 모드 (큰 해상도, 높은 품질)
```bash
ros2 launch econ_image_mosic image_mosaic.launch.py \
    mosaic_width:=1920 \
    mosaic_height:=1080 \
    jpeg_quality:=85 \
    publish_rate:=10.0
```

### 3. 고속 모드 (낮은 해상도, 높은 주파수)
```bash
ros2 launch econ_image_mosic image_mosaic.launch.py \
    mosaic_width:=640 \
    mosaic_height:=480 \
    jpeg_quality:=40 \
    publish_rate:=30.0
```

### 4. PVI 하드웨어 가속 강제 사용
```bash
ros2 launch econ_image_mosic image_mosaic.launch.py \
    use_pvi:=true \
    use_gpu:=false
```

### 5. 비압축 모드 (호환성 최대화)
```bash
ros2 launch econ_image_mosic image_mosaic.launch.py \
    use_compressed:=false \
    mosaic_width:=640 \
    mosaic_height:=480
```

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `mosaic_width` | int | 1280 | 모자이크 이미지 너비 |
| `mosaic_height` | int | 720 | 모자이크 이미지 높이 |
| `publish_rate` | double | 15.0 | 게시 주파수 (Hz) |
| `use_gpu` | bool | true | CUDA GPU 가속 사용 여부 |
| `use_pvi` | bool | true | PVI/NVENC 가속 사용 여부 |
| `jpeg_quality` | int | 60 | JPEG 압축 품질 (1-100) |
| `use_compressed` | bool | true | 압축 이미지 게시 여부 |

## 성능 최적화 가이드

### 메모리 사용량 최적화
1. **압축 모드 사용**: `use_compressed:=true` (기본값)
2. **해상도 조정**: 1280x720 이하 권장
3. **품질 조정**: `jpeg_quality:=40-80` 범위 사용
4. **주파수 조정**: 15-20Hz 권장

### 하드웨어 가속 우선순위
1. **PVI/NVENC** (Jetson 전용, 가장 효율적)
2. **CUDA GPU** (범용 GPU 가속)
3. **CPU** (폴백 모드)

### 메모리풀 오류 해결
```bash
# 메모리풀 설정 확인
cat /tmp/iceoryx_config.toml

# 프로세스 메모리 사용량 모니터링
top -p $(pgrep -f image_mosaic_node)

# 시스템 메모리 확인
free -h
```

## 이미지 확인

### 압축 이미지 확인
```bash
# RViz2로 확인
ros2 run rviz2 rviz2

# 압축 이미지 뷰어로 확인  
ros2 run rqt_image_view rqt_image_view /camera/mosaic/image_raw/compressed
```

### 비압축 이미지 확인
```bash
ros2 run rqt_image_view rqt_image_view /camera/mosaic/image_raw
```

## 토픽 모니터링

```bash
# 게시되는 토픽 목록 확인
ros2 topic list | grep mosaic

# 압축 토픽 정보 확인
ros2 topic info /camera/mosaic/image_raw/compressed

# 토픽 주파수 확인
ros2 topic hz /camera/mosaic/image_raw/compressed

# 토픽 대역폭 확인
ros2 topic bw /camera/mosaic/image_raw/compressed
```

## 문제해결

### 1. 메모리풀 오류 ("Mempool has no more space left")

**원인**: Iceoryx 공유 메모리 부족

**해결방법**:
```bash
# 1. 압축 모드 활성화
ros2 launch econ_image_mosic image_mosaic.launch.py use_compressed:=true

# 2. 해상도 감소
ros2 launch econ_image_mosic image_mosaic.launch.py mosaic_width:=640 mosaic_height:=480

# 3. 품질 조정
ros2 launch econ_image_mosic image_mosaic.launch.py jpeg_quality:=40
```

### 2. QoS 호환성 문제

**원인**: Publisher와 Subscriber의 QoS 설정 불일치

**해결방법**: 노드가 자동으로 BEST_EFFORT QoS를 사용하여 해결됨

### 3. 하드웨어 가속 문제

**PVI 가속 실패시**:
```bash
# CUDA 가속으로 대체
ros2 launch econ_image_mosic image_mosaic.launch.py use_pvi:=false use_gpu:=true

# CPU 모드로 실행
ros2 launch econ_image_mosic image_mosaic.launch.py use_pvi:=false use_gpu:=false
```

### 4. 이미지가 출력되지 않는 경우

```bash
# 1. 입력 토픽 확인
ros2 topic list | grep video

# 2. 카메라 토픽 데이터 확인
ros2 topic hz /dev/video0/image_raw

# 3. 노드 상태 확인
ros2 node info /image_mosaic_node

# 4. 로그 확인
ros2 launch econ_image_mosic image_mosaic.launch.py --ros-args --log-level debug
```

## 성능 벤치마크

| 모드 | 해상도 | 품질 | 메모리 사용량 | CPU 사용률 | 주파수 |
|------|--------|------|---------------|------------|--------|
| 고속 | 640x480 | 40% | ~200MB | ~15% | 30Hz |
| 균형 | 1280x720 | 60% | ~400MB | ~25% | 15Hz |
| 고화질 | 1920x1080 | 85% | ~800MB | ~40% | 10Hz |

## 라이센스

Apache-2.0

## 기여 및 지원

문제 보고나 개선 제안은 이슈 트래커를 통해 제출해주세요. 