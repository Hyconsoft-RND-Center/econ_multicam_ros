# VPI GPU 처리 구현 및 시스템 안정성 개선 작업 기록

## 프로젝트 개요
- **목표**: Jetson AGX Orin에서 ROS2 카메라 드라이버 CPU 사용률 최적화 (50-90% → 15%)
- **플랫폼**: Jetson AGX Orin, JetPack 6.2.0, VPI 3.2
- **기술 스택**: VPI 3.2, CUDA, NVMM, GStreamer, ROS2

## 완료된 작업

### 1. Launch Parameter 파싱 개선
- `encoding:=compressed` 및 `encoding:=BGRx` 파라미터 지원
- VPI GPU 모드 추가: `encoding:=vpi_gpu`
- 토픽 이름 매핑 수정: VPI_GPU 모드에서 `/dev/video%d/image_raw` 토픽 사용

### 2. VPI 3.2 + CUDA 통합 구현
- CMakeLists.txt에 VPI 3.2 및 CUDA 라이브러리 의존성 추가
- 조건부 컴파일 지원 (`HAVE_VPI` 매크로)
- VPI 컨텍스트, 스트림, 이미지 관리 구조체 추가

### 3. GPU 가속 색공간 변환
```c
// VPI 파이프라인 구현
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_VPI_GPU \
    "v4l2src device=/dev/video%d io-mode=2 ! " \
    "video/x-raw,format=UYVY,width=%d,height=%d,framerate=60/1 ! " \
    "nvvidconv ! " \
    "video/x-raw(memory:NVMM),format=NV12,width=%d,height=%d ! " \
    "appsink name=sink sync=false emit-signals=false max-buffers=2 drop=true enable-last-sample=false"
```

### 4. NVMM 버퍼 처리 개선
- NvBufSurface API 사용으로 dmabuf 대신 직접 NVMM 접근
- `NVBUF_MEM_CUDA_UNIFIED` (memType=4) 지원
- `NvBufSurfaceMap/UnMap` 및 `NvBufSurfaceSyncForCpu` 구현

### 5. VPI 네이티브 메모리 관리
```c
// VPI 이미지 생성 및 데이터 복사
VPIImage temp_vpi_image = NULL;
vpiImageCreate(width, height, VPI_IMAGE_FORMAT_NV12_ER, VPI_BACKEND_CUDA, &temp_vpi_image);
vpiImageLockData(temp_vpi_image, VPI_LOCK_WRITE, VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR, &vpi_data);
```

### 6. 시스템 크래시 문제 해결
#### 원인 분석
- **VPI 컨텍스트 불일치**: "Can't use object created in one context with stream created in a different context"
- **메모리 타입 불일치**: NVMM CUDA_UNIFIED를 잘못된 복사 방식으로 처리
- **CUDA 동기화 누락**: GPU 메모리 작업 완료 전 VPI 작업 진행

#### 해결책 구현
1. **VPI 컨텍스트 설정**:
```c
status = vpiContextSetCurrent(publisher->vpi_ctx);
```

2. **메모리 타입별 CUDA 복사**:
```c
enum cudaMemcpyKind copy_kind = (nvbuf_surface->memType == 4) ? 
                               cudaMemcpyDeviceToDevice : cudaMemcpyHostToDevice;
```

3. **CUDA 동기화**:
```c
cudaError_t sync_result = cudaDeviceSynchronize();
```

4. **강화된 에러 처리**: 모든 에러 경로에서 VPI 리소스 정리

### 7. 에러 메시지 개선
- VPI 3.2 API 사용: `vpiGetLastStatusMessage(error_msg, sizeof(error_msg))`
- 상세한 CUDA 에러 정보 제공

## 현재 상태

### 성공한 부분
✅ 시스템 안정성: VPI GPU 처리 시 시스템 다운 문제 해결
✅ VPI 초기화: 모든 카메라에서 "VPI 3.2 GPU processing initialized" 성공
✅ 메모리 안전성: CUDA 메모리 관리 및 VPI 리소스 정리 구현

### 현재 해결 중인 문제
⚠️ **CUDA 복사 오류**: `Failed to copy Y plane: invalid argument`
- NVMM memType=4 (CUDA_UNIFIED) 버퍼에서 VPI CUDA 메모리로 복사 실패
- cudaMemcpy2D 파라미터 검증 필요

## 다음 단계

### 우선순위 1: CUDA 복사 오류 해결
1. NVMM 버퍼 실제 크기 및 정렬 확인
2. VPI CUDA 메모리 포인터 유효성 검증
3. pitch 값과 실제 메모리 레이아웃 불일치 해결

### 성능 최적화 계획
- **현재**: CPU 사용률 50-90%
- **목표**: CPU 사용률 ~15% + GPU 활용
- **전략**: NVMM → VPI CUDA → BGRA8 파이프라인 완성

## 기술적 세부사항

### VPI 3.2 에러 코드 참조
- 에러 코드 6 = `VPI_ERROR_INVALID_OPERATION`
- 컨텍스트 불일치 시 발생하는 대표적 오류

### 메모리 아키텍처
- Jetson AGX Orin: CPU-GPU 공유 메모리 (Unified Memory Architecture)
- NVMM: NVIDIA 멀티미디어 메모리 (Zero-copy 지원)
- VPI: CUDA 백엔드를 통한 GPU 가속 비전 처리

### 빌드 명령어
```bash
colcon build --packages-select econ_ros
```

### 실행 명령어
```bash
# VPI GPU 모드
ros2 launch econ_ros driver.launch.py encoding:=vpi_gpu

# 안전한 대안들
ros2 launch econ_ros driver.launch.py encoding:=BGRx       # CPU 처리
ros2 launch econ_ros driver.launch.py encoding:=compressed # NVMM→JPEG HW 가속
```

## 파일 수정 내역

### 주요 수정 파일
- `src/econ_ros/include/econ_ros/gst_ros_publisher.h`: VPI 구조체 정의
- `src/econ_ros/src/gst_ros_publisher.c`: VPI GPU 처리 로직 구현
- `src/econ_ros/src/main.c`: vpi_gpu 인수 파싱 추가
- `src/econ_ros/launch/driver.launch.py`: vpi_gpu 기본값 설정
- `src/econ_ros/CMakeLists.txt`: VPI 3.2/CUDA 의존성 추가

---

*이 문서는 VPI 3.2 GPU 가속 처리 구현 과정에서 발생한 시스템 안정성 문제 해결과 성능 최적화 작업을 기록합니다.*