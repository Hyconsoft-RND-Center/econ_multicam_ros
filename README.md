# ROS2 driver for ECON Multi-Camera Systems

[ROS2 (/humble/)]

## 개요

이 ROS2 패키지는 V4L2 인터페이스를 사용하는 ECON 멀티카메라 시스템을 지원하며, ROS2 Humble 대상으로 합니다.
실행 시 드라이버는 선택된 카메라 장치들을 설정하고 연결한 후, V4L2를 사용하여 프레임을 캡처하고
카메라별 토픽에 해당하는 ROS 메시지를 발행합니다. 이 드라이버는 다중 카메라를 동시에 지원하며
동기화된 프레임 캡처 기능을 제공합니다.

드라이버는 들어오는 카메라 프레임을 처리하여 ROS2 sensor_msgs/Image 형식으로 변환하고,
`/dev/vide0`, `/dev/video1` ... 등의 하드웨어 이름과 동일한 토픽에 이미지 데이터와 카메라 정보를 발행합니다.

### 드라이버 매개변수

런치 파일이나 매개변수 파일을 통해 설정할 수 있는 주요 매개변수들:

* **num_cam**: 초기화할 카메라 개수 (기본값: 4)
* **width**: 프레임 가로 픽셀 크기 (기본값: 1920)
* **height**: 프레임 세로 픽셀 크기 (기본값: 1080)
* **no_display**: 비디오 디스플레이 비활성화 (기본값: false)
* **record**: 녹화 모드 활성화 (기본값: false)
* **record_time**: 녹화 지속 시간(초) (기본값: 30)
* **disable_sync**: 프레임 동기화 비활성화 (기본값: false)

## 발행되는 토픽

각 카메라에 대해 다음 토픽들이 발행됩니다:

* `/dev/videoN/image_raw` (sensor_msgs/Image)
* `/dev/videoN/camera_info` (sensor_msgs/CameraInfo)

여기서 N은 카메라 인덱스입니다 (0, 1, 2 등).

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
