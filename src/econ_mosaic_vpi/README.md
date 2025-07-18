# VPI 가속 서라운드 뷰 시스템 (econ_surround_vpi)

NVIDIA Jetson AGX Orin에서 VPI 3.2 가속을 사용하여 STURDeCAM31_CUOAGX 모노 카메라 4개로부터 실시간 서라운드 뷰를 생성하는 ROS2 패키지입니다.

## 시스템 요구사항

- **하드웨어**: NVIDIA Jetson AGX Orin
- **OS**: Ubuntu 22.04 Jammy
- **JetPack**: 6.2
- **VPI**: 3.2
- **ROS**: ROS2 Humble
- **카메라**: STURDeCAM31_CUOAGX (4개 - 전, 후, 좌, 우)

## 주요 특징

- **VPI 가속**: NVIDIA VPI 3.2 라이브러리를 사용한 GPU 가속 이미지 처리
- **실시간 처리**: CUDA 백엔드를 통한 고성능 실시간 서라운드 뷰 생성
- **4 카메라 동기화**: message_filters를 사용한 4개 카메라 영상 동기화
- **2x2 그리드 레이아웃**: 차량 주변을 한눈에 볼 수 있는 탑뷰 구성

## 패키지 구조

```
econ_surround_vpi/
├── src/
│   └── surround_view_node.cpp     # 메인 서라운드 뷰 노드
├── launch/
│   └── surround_view.launch.py    # 런치 파일
├── config/
│   └── surround_view_params.yaml  # 설정 파라미터
├── package.xml
├── CMakeLists.txt
└── README.md
```

## 설치 및 빌드

### 1. 의존성 설치

```bash
sudo apt update
sudo apt install libnvvpi3 vpi3-dev
```

### 2. 워크스페이스 빌드

```bash
cd /home/hycon/econ_surround_ws
colcon build --packages-select econ_surround_vpi --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3. 환경 설정

```bash
source /home/hycon/econ_surround_ws/install/setup.bash
```

## 실행 방법

### 1. 기본 실행

```bash
ros2 launch econ_surround_vpi surround_view.launch.py
```

### 2. 파라미터 커스터마이징

```bash
ros2 launch econ_surround_vpi surround_view.launch.py \
    output_width:=1024 \
    output_height:=768 \
    camera_width:=1920 \
    camera_height:=1080
```

### 3. 수동 노드 실행

```bash
ros2 run econ_surround_vpi surround_view_node
```

## 카메라 토픽 구성

기본적으로 다음 토픽들을 구독합니다:

- `/dev/video0/image_raw` → 전방 카메라 (Front)
- `/dev/video1/image_raw` → 후방 카메라 (Rear)  
- `/dev/video2/image_raw` → 좌측 카메라 (Left)
- `/dev/video3/image_raw` → 우측 카메라 (Right)

## 출력 토픽

- `/surround_view/image_raw`: 합성된 서라운드 뷰 이미지 (BGRx8 형식)

## 서라운드 뷰 레이아웃

```
┌─────────┬─────────┐
│  Left   │  Front  │  ← 상단
│ Camera  │ Camera  │
├─────────┼─────────┤
│  Rear   │ Right   │  ← 하단
│ Camera  │ Camera  │
└─────────┴─────────┘
```

## 파라미터 설정

### 주요 파라미터

- `output_width`: 출력 서라운드 뷰 이미지 폭 (기본값: 800)
- `output_height`: 출력 서라운드 뷰 이미지 높이 (기본값: 600)
- `camera_width`: 입력 카메라 이미지 폭 (기본값: 1920)
- `camera_height`: 입력 카메라 이미지 높이 (기본값: 1080)

### 고급 설정

`config/surround_view_params.yaml` 파일에서 다음을 설정할 수 있습니다:

- VPI 백엔드 설정 (CUDA, VIC, CPU)
- 보간 모드 (LINEAR, NEAREST)
- 동기화 파라미터
- 색상 보정 설정

## VPI 성능 최적화

### CUDA 백엔드 사용

현재 구현은 CUDA 백엔드를 사용하여 GPU에서 이미지 처리를 수행합니다:

- **이미지 리사이징**: `vpiSubmitRescale` (CUDA 가속)
- **이미지 변환**: VPI 이미지 포맷 최적화
- **메모리 관리**: 제로카피 메모리 매핑

### 성능 모니터링

```bash
# VPI 성능 확인
jtop

# ROS2 토픽 주파수 모니터링  
ros2 topic hz /surround_view/image_raw
```

## 시각화

### RViz2에서 확인

```bash
# RViz2 실행
rviz2

# 이미지 토픽 추가: /surround_view/image_raw
```

### rqt_image_view 사용

```bash
ros2 run rqt_image_view rqt_image_view /surround_view/image_raw
```

## 문제 해결

### 1. VPI 라이브러리 오류

```bash
# VPI 설치 확인
ls -la /opt/nvidia/vpi3/

# 환경변수 설정
export LD_LIBRARY_PATH=/opt/nvidia/vpi3/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
```

### 2. 카메라 토픽 누락

```bash
# 카메라 토픽 확인
ros2 topic list | grep camera

# 카메라 드라이버 상태 확인
ros2 topic info /camera0/image_raw
```

### 3. 메모리 부족

- `output_width`, `output_height` 파라미터를 줄여보세요
- 시스템 메모리 사용량을 확인하세요: `free -h`

## 개발 및 확장

### 왜곡 보정 추가

향후 어안 렌즈 왜곡 보정을 위해 다음을 추가할 수 있습니다:

```cpp
// 카메라 캘리브레이션 파라미터 로드
// vpiSubmitLensDistortionCorrection 사용
```

### 추가 VPI 알고리즘

- 템포럴 노이즈 감소 (TNR)
- 히스토그램 균등화
- 엣지 검출 필터

## 라이센스

Apache 2.0

## 기여

버그 리포트나 개선사항은 이슈로 등록해 주세요.

## 참고 자료

- [NVIDIA VPI 3.2 문서](https://docs.nvidia.com/vpi/)
- [JetPack 6.2 릴리즈 노트](https://developer.nvidia.com/jetpack)
- [STURDeCAM31 카메라 사양](https://www.e-consystems.com/nvidia-cameras/jetson-agx-orin-cameras/sony-isx031-ip69k-hdr-gmsl2-camera.asp) 