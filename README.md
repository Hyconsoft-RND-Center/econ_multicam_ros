# ECON Multi-Camera ROS2 Driver (GStreamer GPU 버전)

> **Branch:** `gstreamer`  |  **ROS2 Distro:** Humble  |  **Platform:** Jetson Orin / Xavier / Nano

이 브랜치는 Jetson 하드웨어 코덱·VIC·NVJPG를 활용해 **Zero-copy GPU 파이프라인**으로 멀티카메라 스트림을 ROS2 토픽에 발행합니다. 기존 V4L2 캡처 루프를 제거하고 `v4l2src → nvvidconv → nvjpegenc` 등 GStreamer 요소가 직접 카메라 버퍼(NVMM)를 처리하므로 CPU 부하를 대폭 줄였습니다.

---

## 특징
* `v4l2src` 기반 **Zero-copy** 캡처 (DMA-BUF)
* HW JPEG 인코더(`nvjpegenc`) / VIC 컨버터(`nvvidconv`) 사용
* Iceoryx 4 MB 한계를 회피하기 위한 **sensor_msgs/CompressedImage** 지원
* 카메라 수만큼 독립 파이프라인을 생성·관리 (스레드 X)
* CPU 사용률 < 10 % (4×1080p@30 Hz 기준)

---

## 의존 패키지
```bash
sudo apt update
sudo apt install -y \
  gstreamer1.0-tools gstreamer1.0-plugins-{base,good,bad} \
  nvidia-l4t-gstreamer # JetPack 기본 포함 여부 확인
```

---

## 빌드
```bash
cd <workspace>
colcon build --symlink-install --packages-select econ_ros
source install/setup.bash
```

---

## 실행 예시
```bash
ros2 run econ_ros econ_multicam_node \
  --ros-args -p num_cam:=4 -p width:=1920 -p height:=1080
```
*카메라 `/dev/video0–3` 가 1080p30 으로 JPEG 압축되어 퍼블리시 됩니다.*

---

## 매개변수
| 이름 | 설명 | 기본값 |
|------|------|--------|
| `num_cam` | 카메라 개수 | `4` |
| `width` / `height` | 프레임 해상도 | `1920` / `1080` |
| `jpeg_quality` | HW JPEG 품질 (0–100) | `85` |
| `framerate` | 출력 FPS | `30` |

---

## 발행 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/dev/videoN/image_raw/compressed` | `sensor_msgs/CompressedImage` | HW JPEG 비트스트림 |
| `/dev/videoN/image_raw` | `sensor_msgs/Image` | (옵션) JPEG 디코딩 후 RGB |
| `/dev/videoN/camera_info` | `sensor_msgs/CameraInfo` | 카메라 intrinsic |

`rqt_image_view` 사용 시 `…/compressed` 선택 → 즉시 미리보기 가능. 플러그인 미설치 시:
```bash
sudo apt install ros-humble-compressed-image-transport
```

---

## 성능 (Jetson Orin NX, 4×1080p@30 Hz)
| 지표 | 값 |
|------|----|
| GR3D | 0 % |
| VIC | 30–70 % |
| NVJPG | 10–20 % |
| CPU (A78) | 5–8 % |

> GPU/VIC 사용률은 `tegrastats` 로 확인하세요. (`nvidia-smi` 미지원)

---
