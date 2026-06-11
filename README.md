# 🤖 hot-spot_isaac_sim

**hot-spot_isaac_sim**은 NVIDIA Isaac Sim 시뮬레이션 환경과 ROS 2 Humble Nav2 내비게이션 스택을 결합하여, 사족보행 로봇 Spot 2대의 협업을 바탕으로 화재를 자동 진압하고 인명을 안전하게 대피시키는 재난 대응 자동화 시뮬레이션 프로젝트입니다.

---

## 🌟 주요 기능

### 1. robot1 (소화기 화재 진압 로봇)
*   **화재 자동 탐지**: 화재 센서(10초 경과 시)로부터 점화 상태를 받아 자동으로 행동을 개시합니다.
*   **소화기 자동 파지 (Grasp)**: Nav2를 통해 소화기(`Cube`)가 배치된 지점(`(9.83, 0.5)`)으로 이동한 뒤, 기하학적 로봇 팔 조작으로 소화기를 파지합니다.
*   **화재 구역 이동 및 투척 (Throw)**: 소화기를 든 채 화재 지점(`(-4.93, -5.78)`)으로 자율 주행하여 적절한 진압 거리에서 소화기를 자동으로 투척합니다.
*   **소화 효과 렌더링**: 소화기가 불길 근처 바닥에 닿으면 하얀 가스 효과(ExtinguisherGas)를 방출하고, 불길(Flow Fire)의 크기를 점차 줄여 완전 소화합니다.
*   **안전 대피**: 진압 임무를 완료한 뒤 건물 밖 대피소(`(1.568, 20.041)`)로 자동 대피합니다.

### 2. robot2 (자율 순찰 및 인명 구조 로봇)
*   **격실 자율 순찰**: 기동 직후 5번방 ➔ 1번방 ➔ 3번방 ➔ 4번방 ➔ 6번방 순서대로 자율주행하고, 각 방에 도착할 때마다 제자리에서 360도 회전(Spin)하며 사각지대를 정밀 수색합니다.
*   **YOLOv8 기반 인명 탐지**: 그리퍼에 장착된 카메라 프레임을 실시간 비전 처리(`yolov8n.pt`)하여 사람(`person`)을 탐지합니다.
*   **조난자 접근 및 정렬**: 사람이 감지되면 순찰 경로를 즉시 이탈(Cancel Goal)하고, 화면 중앙 정렬 및 센서 기반 거리 측정을 수행하여 조난자 앞 1.15m 거리까지 정밀 접근합니다.
*   **안전 호송 (Escort)**: 조난자 충돌체를 일시 비활성화하고 Costmap을 비워 병목 현상을 방지한 뒤, 출구(`(1.568, 20.041)`)로 자율 이동합니다. 이때 조난자는 로봇 후방 2.4m 간격을 유지하며 뒤따라오도록 시뮬레이션됩니다.
*   **탈출 완료**: 조난자를 이끌고 안전하게 건물 외부로 탈출을 완료합니다.

### 3. 중앙 관제 및 모니터링 (RViz2)
*   실시간 로봇 위치 및 이동 궤적 시각화
*   라이다 스캔(Functional Lidar) 및 빌드된 맵 상의 수색 경로 표출
*   실시간 로봇 자세 상태(MarkerArray) 가시화

---

## 🗺️ 시스템 설계 및 플로우 차트 (다이어그램)

본 프로젝트의 아키텍처 및 상세 흐름은 다음 다이어그램 문서들을 참조하십시오. (각 링크 클릭 시 Mermaid 렌더러로 작성된 시각적 흐름도를 보실 수 있습니다.)

1.  **[① 전체 시스템 아키텍처 다이어그램 (Overall System Architecture)](docx/1_overall_system_architecture.md)**
    *   Isaac Sim, ROS 2 Nav2, AI/RL 모델 및 브릿지 스크립트 간의 전체 결합도 및 데이터 연동 흐름.
2.  **[② 쉘 스크립트 실행 파이프라인 흐름도 (Execution Stage Pipeline Flowchart)](docx/2_execution_stage_pipeline.md)**
    *   Stage 1(수동 확인)부터 Stage 5(완전 자동화)까지 실행 단계별 기동 노드 및 시나리오 확장 구조.
3.  **[③ 통신 브릿지 네트워크 다이어그램 (Network & Bridge Communication Diagram)](docx/3_communication_bridge_network.md)**
    *   ROS 2 cmd_vel/plan/odom 토픽과 Isaac Sim 소켓/파일 I/O 간의 실시간 브릿지 통신 메커니즘.
4.  **[④ 멀티 로봇 Nav2 노드 및 TF 트리 아키텍처 (Multi-Robot Nav2 & TF Tree Architecture)](docx/4_multi_robot_nav2_tf.md)**
    *   로봇별 독립 네임스페이스(`robot1`, `robot2`) 분리, 공유 맵 서버, 글로벌 TF 트리 통합 구조.
5.  **[⑤ 로봇 정찰/순찰 임무 시퀀스 다이어그램 (Room Patrol Mission Sequence Diagram)](docx/5_room_patrol_sequence.md)**
    *   순회 경로 순찰 ➔ 360도 스핀 동작 및 YOLOv8 조난자 발견 시의 자율주행 인터럽트 천이 과정.
6.  **[⑥ Spot Arm RL Policy 루프 플로우차트 (Spot Arm Policy Action-State Loop)](docx/6_spot_arm_policy_loop.md)**
    *   69차원 관측(Observation) ➔ JIT/MLP 정책 추론 ➔ 19차원 Action 및 동작 강성(Stiffness) 제어 루프.
7.  **[⑦ 객체 인식 및 맵핑 통합 시퀀스 (YOLOv8 Object Detection & Mapping Flow)](docx/7_yolov8_detection_mapping.md)**
    *   YOLOv8 인명 검출에 따른 상태 머신(`SEARCHING` ➔ `ALIGNING` ➔ `APPROACHING` ➔ `ESCORTING` ➔ `DONE`) 전환 흐름.
8.  **[⑧ 전체 서비스 시나리오 플로우차트 (End-to-End Service Scenario Flowchart)](docx/8_end_to_end_service_scenario.md)**
    *   관리자 관제 시작 ➔ 화재 경보 ➔ 로봇 협업 진압/구조 ➔ 안전 복귀 재난 통제 완료 비즈니스 밸류 체인.

---

## 💻 운영체제 환경

*   **Host OS**: Windows 10/11 Professional (Isaac Sim 시뮬레이터 구동 환경)
*   **Robot OS**: Ubuntu 22.04 LTS / ROS 2 Humble Hawksbill (로봇 브레인 및 Nav2 자율주행 제어 환경)
*   **ROS Middleware (RMW)**: `rmw_cyclonedds_cpp`
*   **Python 버전**: Python 3.10 / 3.11

---

## 🛠️ 사용한 장비 목록

*   **사족보행 로봇**: Boston Dynamics Spot (가상 USD 에셋: `spot_arm.usd`)
*   **로봇 매니퓰레이터**: Spot Arm (물리 그리퍼 조작 및 소화기 파지 담당)
*   **라이다 센서**: Functional Lidar (2D SLAM 및 위치 추정 목적)
*   **비전 카메라**: RGB-D 그리퍼 카메라 (YOLOv8 분석용 비전 피드 공급)
*   **연산 H/W**: 고성능 GPU 탑재 워크스테이션 (NVIDIA GeForce RTX 3080 / 4080 이상 권장, Omniverse Isaac Sim 실시간 렌더링 목적)

---

## 📦 의존성 (requirements.txt)

본 프로젝트는 다음 패키지들을 기반으로 동작합니다. 로봇 환경에서 아래 명령어를 수행하여 패키지를 설치해 주십시오.

```bash
pip install -r requirements.txt
```

**[requirements.txt](requirements.txt)** 내용:
```text
numpy
torch
ultralytics
pyyaml
```

---

## 🚀 실행 순서 (Launch 순서 및 스크립트)

완전 자동화 재난 대응 시나리오(Stage 5)를 기동하기 위한 세부 가이드입니다.

### 1단계: Isaac Sim 시뮬레이션 가동 (터미널 1)
시뮬레이터 내부 물리 월드와 2대의 로봇 에이전트, 화재/소화 이펙트를 가동하고 속도 통신용 UDP 소켓 서버를 엽니다.
```bash
cd ~/cobot_fire_sim
./run_stage5.sh
```
*   *내부적 실행*: Isaac Sim의 가상 파이썬 환경을 통해 `applications/main_simulation.py`를 실행하며, `cmd_vel_udp_bridge.py` 서브프로세스를 동시 실행합니다.

### 2단계: ROS 2 Humble Nav2 및 멀티 로봇 런치 가동 (터미널 2)
공통 맵 서버를 띄우고, 각 로봇의 파라미터를 적용한 독립 자율주행 노드 스택과 위치/경로 중계 브릿지를 가동합니다.
```bash
cd ~/cobot_fire_sim
source /opt/ros/humble/setup.bash
ros2 launch run_multi_nav2.launch.py
```
*   *내부적 실행*:
    1.  `nav2_map_server`를 기동하여 공통 맵(`nav2_map.yaml`) 발행.
    2.  `pose_file_to_ros_bridge.py` (실시간 로봇 위치 ➔ Odom/TF 중계) 실행.
    3.  `robot2_room_patrol.py` (방 순환 순찰 관리 노드) 및 `robot2_plan_file_bridge.py` (경로 계획 전달) 실행.
    4.  `robot1`, `robot2` 네임스페이스별 `navigation_launch.py` 기동.
    5.  통합 관제창 `rviz2` 실행 (`multi_robot_nav2.rviz` 파일 로드).

### 3단계: 순찰 및 비상 시나리오 전개
*   터미널 2에서 런치가 정상 활성화되면 `robot2` 순찰 노드가 대기합니다.
*   10초 후 화재가 발생하면, `robot1`이 화재를 감지하고 자동으로 소화기로 이동해 파지 후 화재 구역으로 가져가 불을 끕니다.
*   자율주행 환경에서 `robot2`에 대해 `/robot2/start_room_patrol` 신호가 인가되면 격실 순찰을 돌며 조난자를 발견해 외부로 유도하고, 최종적으로 두 로봇 모두 무사히 복귀하며 작전이 종료됩니다.
