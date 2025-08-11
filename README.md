# ECON Multi-Camera ROS2 Driver

GStreamer 기반 멀티카메라 스트리밍을 지원하는 ROS2 드라이버입니다.

## 개요

이 패키지는 ECON 멀티카메라 시스템을 ROS2 환경에서 사용할 수 있도록 하는 드라이버입니다. GStreamer를 활용하여 효율적인 카메라 스트리밍을 제공하며, 여러 인코딩 방식을 지원합니다.

## 지원 플랫폼
- **ROS2 Distro:** Humble
- **Platform:** Linux (특히 Jetson 플랫폼 최적화)
- **Hardware:** V4L2 호환 멀티카메라 시스템

## 특징
- GStreamer 기반 카메라 스트리밍
- 다중 인코딩 방식 지원 (BGRx, Compressed, JPEG)
- 멀티카메라 동기화 지원
- ROS2 launch 파일을 통한 쉬운 설정

## 의존 패키지

### 시스템 패키지
```bash
sudo apt update
sudo apt install -y \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev
```

### ROS2 패키지
```bash
sudo apt install -y \
  ros-humble-sensor-msgs \
  ros-humble-rclc \
  ros-humble-std-msgs
```

## 빌드

```bash
cd your_workspace
colcon build --packages-select econ_ros
source install/setup.bash
```

## 실행 방법

### Launch 파일 사용 (권장)
```bash
# 기본 실행 (BGRx 인코딩)
ros2 launch econ_ros driver.launch.py

# Compressed/JPEG 인코딩
ros2 launch econ_ros driver.launch.py encoding:=compressed

# 해상도 설정
ros2 launch econ_ros driver.launch.py width:=1280 height:=720

# 카메라 개수 제한
ros2 launch econ_ros driver.launch.py num_cam:=2
```

### 직접 실행
```bash
ros2 run econ_ros econ_ros -w 1920 -h 1080 -e compressed
```

## 매개변수

| 매개변수 | 설명 | 기본값 | 옵션 |
|---------|------|--------|------|
| `width` | 프레임 너비 | 1920 | 640~1920 |
| `height` | 프레임 높이 | 1080 | 480~1080 |
| `encoding` | 인코딩 방식 | BGRx | BGRx, compressed, jpeg |
| `no_display` | 디스플레이 비활성화 | 1 | 0, 1 |
| `record` | 녹화 활성화 | 0 | 0, 1 |
| `sync` | 프레임 동기화 | 1 | 0, 1 |

## 발행 토픽

각 카메라별로 다음 토픽들이 발행됩니다:

| 토픽 패턴 | 메시지 타입 | 설명 |
|----------|-------------|------|
| `/dev/videoN/image_raw` | `sensor_msgs/Image` | 원본 이미지 데이터 |
| `/dev/videoN/camera_info` | `sensor_msgs/CameraInfo` | 카메라 보정 정보 |

여기서 N은 카메라 번호입니다 (0, 1, 2, ...).

## 인코딩 방식

### BGRx (기본값)
- 원시 BGRA8 형식
- 호환성이 좋음
- CPU 사용량 높음

### Compressed/JPEG
- JPEG 압축 이미지
- 네트워크 대역폭 절약
- 하드웨어 가속 지원 (Jetson)

## 사용 예시

### 이미지 확인
```bash
# 원시 이미지 확인
ros2 topic echo /dev/video0/image_raw

# 토픽 리스트 확인
ros2 topic list | grep video
```

### rqt를 통한 이미지 뷰
```bash
rqt_image_view
```

## 문제 해결

### 카메라 장치 확인
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

### GStreamer 플러그인 확인
```bash
gst-inspect-1.0 | grep v4l2
```

## 라이선스

Apache License 2.0