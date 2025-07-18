# ECON Image Processing Workspace

## 개요
STURDeCAM31 카메라를 위한 이미지 처리 ROS2 패키지들의 워크스페이스입니다.

## 패키지

### econ_mosaic_vpi
- **설명**: VPI 가속 4카메라 모자이크 뷰 시스템
- **기능**: 
  - 4개 카메라 이미지를 2x2 그리드로 합성
  - NVIDIA VPI CUDA 백엔드 가속 
  - ROS2 메시지 동기화
  - 실시간 이미지 처리
- **토픽**: 
  - 입력: `/dev/video0-3/image_raw`
  - 출력: `/mosaic_view/image_raw`

### econ_surround  
- **설명**: Python 기반 서라운드 뷰 처리 패키지
- **상태**: 개발 중

## 시스템 요구사항
- **하드웨어**: NVIDIA Jetson AGX Orin
- **소프트웨어**: 
  - JetPack 6.2
  - VPI 3.2
  - ROS2 Humble
  - Ubuntu 22.04 Jammy
- **카메라**: STURDeCAM31_CUOAGX x4 (전면/후면/좌측/우측)

## 빌드 및 실행

### 빌드
```bash
cd econ_image_proc_ws
colcon build --symlink-install
source install/setup.bash
```

### 실행
```bash
# 모자이크 뷰 시스템 실행
ros2 launch econ_mosaic_vpi mosaic_view.launch.py
```

## 설정
- 파라미터 설정: `src/econ_mosaic_vpi/config/params.yaml`
- 출력 해상도: 800x600 (기본값)
- 입력 해상도: 1920x1080 (기본값)

## 개발 히스토리
- 초기 버전: VPI 가속 모자이크 뷰 시스템 구현
- git 저장소 초기화: 2025-07-18 