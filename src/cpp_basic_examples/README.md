# ROS 2 C++ 기초 예제

ROS 2 공식 튜토리얼([Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html))를 바탕으로 한 퍼블리셔/서브스크라이버 예제입니다.  
ROS 2 노드 구조, 토픽 통신, `colcon` 빌드 방법을 익히는 데 활용하세요.

## 준비 사항
- ROS 2 Humble 이상 또는 Foxy/ Galactic 버전 (패키지 XML은 `ament_cmake`를 사용)
- 워크스페이스 루트: `~/ros2_ws`

## 빌드 방법
```bash
cd ~/ros2_ws
colcon build --packages-select cpp_basic_examples
source install/setup.zsh  # bash 사용 시 setup.bash
```

## 실행 방법
- 터미널 1: 퍼블리셔 노드 실행
  ```bash
  ros2 run cpp_basic_examples basic_publisher
  ```
- 터미널 2: 서브스크라이버 노드 실행
  ```bash
  ros2 run cpp_basic_examples basic_subscriber
  ```

두 노드가 정상적으로 실행되면 `greetings` 토픽을 통해 문자열 메시지가 전송되고, 퍼블리셔는 발행 로그를, 서브스크라이버는 수신 로그를 출력합니다.

## 코드 개요
- `src/basic_publisher.cpp`
  - 0.5초마다 `std_msgs::msg::String` 메시지를 생성해 `greetings` 토픽으로 발행합니다.
  - `rclcpp::create_publisher`와 `create_wall_timer` 사용법을 익힐 수 있습니다.
- `src/basic_subscriber.cpp`
  - 동일한 토픽을 구독하여 전달된 메시지를 로그로 출력합니다.
  - 콜백 함수와 `create_subscription` 사용 예시가 포함되어 있습니다.

## 추가 학습 포인트
1. QoS 설정 변경: `rclcpp::QoS`를 사용해 신뢰성, 히스토리를 조정해 보세요.
2. 커스텀 메시지: `ros2 interface show std_msgs/msg/String` 명령으로 인터페이스 정의를 확인하고, 자신의 메시지를 만들 수 있습니다.
3. 노드 실행 옵션: `ros2 run` 명령에 `--ros-args -r greetings:=new_topic` 과 같이 리매핑 옵션을 적용해 토픽 이름을 바꿔보세요.
4. Launch 파일: 복수의 노드를 동시에 실행하려면 Python 기반 `launch` 파일을 만들어 자동화할 수 있습니다.

이 예제를 기반으로 ROS 2의 기본 통신 구조에 익숙해지면, 서비스/액션, 파라미터 서버 등 다른 요소로 학습 범위를 넓혀보세요.
