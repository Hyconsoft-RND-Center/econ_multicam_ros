# econ_image_mosic

4개의 카메라 이미지를 하나의 모자이크 이미지로 결합하는 ROS2 패키지입니다. **VPI 3.2 JetPack 6.2.0 최적화**를 통해 Jetson AGX Orin의 하드웨어 가속을 최대 활용하여 고성능 이미지 처리를 제공합니다.

## 주요 특징

- **VPI 3.2 최적화**: Jetson AGX Orin 하드웨어 가속 최대 활용
- **자동 백엔드 감지**: CUDA → VIC → PVA → CPU 우선순위 자동 선택
- **빠른 시작**: 시작 시간 대폭 단축 (42초 → 수 초)
- **비동기 초기화**: VPI와 GStreamer 백그라운드 초기화
- **메모리 최적화**: Iceoryx 메모리풀 부족 문제 해결
- **압축 전송**: JPEG 압축으로 메모리 사용량 대폭 감소
- **실시간 처리**: 20-30Hz 고성능 모자이크 생성
- **자동 리소스 관리**: 효율적인 메모리 및 GPU 리소스 관리

## 시스템 요구사항

### Jetson 플랫폼 (권장)
- **NVIDIA Jetson AGX Orin** (최적화됨) 
- **JetPack 6.2.0** (VPI 3.2 포함)
- **Ubuntu 22.04 LTS**
- **최소 8GB RAM**

### 지원 백엔드
| 백엔드 | 플랫폼 | 성능 | 상태 |
|--------|--------|------|------|
| **CUDA** | Jetson AGX Orin | 최고 | 자동 감지 |
| **VIC** | Jetson AGX Orin | 높음 | 자동 감지 |
| **PVA** | Jetson AGX Orin | 중간 | 자동 감지 |
| **CPU** | 모든 플랫폼 | 기본 | 폴백 |

### 필수 의존성
- ROS2 Humble
- VPI 3.2+ (JetPack 6.2.0 포함)
- OpenCV 4.x
- GStreamer (nvjpegenc 플러그인)

## 입력/출력 토픽

### 입력 토픽
- `/dev/video0/image_raw` (sensor_msgs/Image)
- `/dev/video1/image_raw` (sensor_msgs/Image)  
- `/dev/video2/image_raw` (sensor_msgs/Image)
- `/dev/video3/image_raw` (sensor_msgs/Image)

### 출력 토픽
- `/camera/mosaic/image_raw/compressed` (sensor_msgs/CompressedImage) - **기본값, 권장**
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

## 설치 및 빌드

### 1. JetPack 6.2.0 확인
```bash
# JetPack 버전 확인
cat /etc/nv_tegra_release

# VPI 3.2 확인
python3 -c "import vpi; print('VPI 버전:', vpi.__version__)"
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

### 1. 기본 실행 (VPI 3.2 최적화, 권장)
```bash
ros2 launch econ_image_mosic image_mosaic_hw.launch.py
```

**특징**:
- ✅ VPI 3.2 자동 백엔드 감지
- ✅ CUDA/VIC 하드웨어 가속 우선 사용
- ✅ 빠른 시작 (수 초 내 토픽 발행)
- ✅ 비동기 초기화로 응답성 최대화

### 2. 고화질 모드
```bash
ros2 launch econ_image_mosic image_mosaic_hw.launch.py \
    mosaic_width:=1920 \
    mosaic_height:=1080 \
    jpeg_quality:=85 \
    publish_rate:=20.0
```

### 3. 고속 모드
```bash
ros2 launch econ_image_mosic image_mosaic_hw.launch.py \
    mosaic_width:=640 \
    mosaic_height:=480 \
    jpeg_quality:=60 \
    publish_rate:=30.0
```

### 4. 레거시 모드 (CPU만 사용)
```bash
ros2 launch econ_image_mosic image_mosaic.launch.py
```

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `mosaic_width` | int | 1280 | 모자이크 이미지 너비 |
| `mosaic_height` | int | 720 | 모자이크 이미지 높이 |
| `publish_rate` | double | 30.0 | 게시 주파수 (Hz) |
| `jpeg_quality` | int | 85 | JPEG 압축 품질 (1-100) |
| `use_compressed` | bool | true | 압축 이미지 게시 여부 |

## 성능 벤치마크

### Jetson AGX Orin (JetPack 6.2.0)
| 모드 | 백엔드 | 해상도 | 실제 FPS | 시작 시간 | GPU 활용도 |
|------|--------|--------|----------|-----------|------------|
| **VPI 3.2 최적화** | CUDA | 1280x720 | **21.4Hz** | **~3초** | **활성화** |
| **VPI 3.2 최적화** | VIC | 1280x720 | **19.8Hz** | **~3초** | **활성화** |
| **VPI 3.2 최적화** | PVA | 1280x720 | **18.5Hz** | **~3초** | **활성화** |
| 레거시 (CPU만) | CPU | 1280x720 | 15.2Hz | ~42초 | 미사용 |

### 메모리 사용량
| 해상도 | 압축 품질 | 메모리 사용량 | 대역폭 |
|--------|-----------|---------------|--------|
| 640x480 | 60% | ~150MB | ~2MB/s |
| 1280x720 | 85% | ~300MB | ~5MB/s |
| 1920x1080 | 85% | ~600MB | ~12MB/s |

## VPI 3.2 최적화 기능

### 자동 백엔드 감지
```bash
# 로그에서 확인 가능
[INFO] CUDA 백엔드 감지 성공
[INFO] VIC 백엔드 감지 성공
[INFO] VPI-CUDA 백엔드 선택 (주백엔드)
```

### 비동기 초기화
- **VPI**: 백그라운드에서 백엔드 감지 및 초기화
- **GStreamer**: 병렬로 nvjpegenc 파이프라인 초기화
- **결과**: 전체 시작 시간 대폭 단축

### 백엔드 우선순위 (Jetson AGX Orin)
1. **CUDA** - 최고 성능, GPU 활용 최대화
2. **VIC** - 전용 하드웨어, 전력 효율성
3. **PVA** - 프로그래머블 비전 가속기
4. **CPU** - 폴백 모드, 호환성 최대화

## 이미지 확인

### 압축 이미지 확인 (권장)
```bash
# RViz2로 확인
ros2 run rviz2 rviz2

# 실시간 뷰어
ros2 run rqt_image_view rqt_image_view /camera/mosaic/image_raw/compressed
```

### 토픽 모니터링
```bash
# VPI 상태 확인
ros2 topic echo /camera/mosaic/image_raw/compressed --once | head -5

# 성능 모니터링
ros2 topic hz /camera/mosaic/image_raw/compressed
ros2 topic bw /camera/mosaic/image_raw/compressed
```

## 문제해결

### 1. VPI 백엔드 감지 실패
**증상**: `[WARN] VPI 백엔드 감지 실패, CPU 강제 활성화`

**원인**: VPI 3.2 설치 문제 또는 CUDA 드라이버 이슈

**해결방법**:
```bash
# 1. JetPack 6.2.0 재설치 확인
sudo apt update && sudo apt install nvidia-jetpack

# 2. VPI 3.2 설치 확인
python3 -c "import vpi; print('VPI 3.2 설치됨:', vpi.__version__)"

# 3. CUDA 상태 확인  
nvidia-smi  # 또는 tegrastats

# 4. 재부팅 후 재시도
sudo reboot
```

### 2. 시작 시간이 여전히 긴 경우
**원인**: 비동기 초기화 실패

**해결방법**:
```bash
# 로그 레벨을 올려서 상세 확인
ros2 launch econ_image_mosic image_mosaic_hw.launch.py --ros-args --log-level debug
```

### 3. GPU 사용률이 0%인 경우
**원인**: CPU 백엔드만 사용됨

**해결방법**:
```bash
# VPI 백엔드 상태 확인
ros2 topic echo /rosout --once | grep "VPI 백엔드"

# 강제 재시작
sudo systemctl restart nvargus-daemon
ros2 launch econ_image_mosic image_mosaic_hw.launch.py
```

### 4. 메모리풀 오류
**증상**: `"Mempool has no more space left"`

**해결방법**:
```bash
# Iceoryx 설정 최적화 (자동 적용됨)
# 또는 해상도 조정
ros2 launch econ_image_mosic image_mosaic_hw.launch.py \
    mosaic_width:=640 mosaic_height:=480
```