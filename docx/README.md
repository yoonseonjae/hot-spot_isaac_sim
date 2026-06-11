# 📊 Cobot Fire Sim 시스템 다이어그램 가이드

본 폴더는 **Cobot Fire Sim** 프로젝트의 주요 아키텍처, 네트워크 통신, 멀티 로봇 제어, AI/RL 구동 방식 및 서비스 시나리오를 시각화한 다이어그램들을 제공합니다. 

모든 다이어그램은 **Mermaid** 문법으로 작성되었으며, GitHub에서 마크다운(.md) 파일 형태로 즉시 렌더링되어 조회할 수 있습니다. 각 다이어그램은 상세한 구조적 매핑과 함께 코드 베이스(`applications/`, `bridges/`, `launch/` 등)와의 매핑을 명시하고 있으며, 주요 설명은 이해하기 쉽도록 한글로 표시하였습니다.

---

## 📂 다이어그램 목차 및 링크

### 🏗️ 1. 전체 시스템 및 인프라 (System & Infrastructure)
*   **[① 전체 시스템 아키텍처 다이어그램 (Overall System Architecture)](1_overall_system_architecture.md)**
    *   *내용*: Isaac Sim 시뮬레이터, ROS 2 Nav2 내비게이션, AI/RL 기반 Spot 정책 모델, 그리고 관제 시스템 간의 전체적인 연결 관계와 데이터 흐름을 한눈에 조망합니다.
*   **[② 쉘 스크립트 실행 파이프라인 흐름도 (Execution Stage Pipeline Flowchart)](2_execution_stage_pipeline.md)**
    *   *내용*: `run_stage1.sh`부터 `run_stage5.sh`까지 각 구동 단계별로 켜지는 환경 요소, 로봇 동작 범위, AI 추론 활성화 여부 등을 체계적으로 도식화합니다.

### 🌐 2. 통신 및 네트워크 (Communication & Bridge)
*   **[③ 통신 브릿지 네트워크 다이어그램 (Network & Bridge Communication)](3_communication_bridge_network.md)**
    *   *내용*: ROS 2 토픽/액션 생태계와 외부 프로세스(또는 Isaac Sim 내부) 간의 데이터를 중계하는 Python 브릿지(`cmd_vel_udp_bridge.py`, `pose_file_to_ros_bridge.py`, `robot2_plan_file_bridge.py`)의 포트 및 파일 I/O 통신 구조를 정의합니다.

### 🤖 3. 자율주행 및 멀티 로봇 (Multi-Robot Navigation)
*   **[④ 멀티 로봇 Nav2 노드 및 TF 트리 아키텍처 (Multi-Robot Nav2 & TF Tree)](4_multi_robot_nav2_tf.md)**
    *   *내용*: `robot1`과 `robot2` 네임스페이스 분할에 따른 파라미터 매핑과 단일 `map_server` 공유, 그리고 RViz2 시각화 트리에서 두 로봇의 관절 및 센서가 global `map` 좌표계에 머징되는 TF 구조를 명시합니다.
*   **[⑤ 로봇 정찰/순찰 임무 시퀀스 다이어그램 (Room Patrol Mission Sequence)](5_room_patrol_sequence.md)**
    *   *내용*: `robot2_room_patrol.py` 노드가 방들을(5 ➔ 1 ➔ 3 ➔ 4 ➔ 6) 순환하는 자율 순찰 프로세스를 담당하며, 각 목표지점 도달 시 수행하는 제자리 회전(Spin) 액션 및 사람 발견 시 시나리오 중단/구조 모드 전환을 다룹니다.

### 🧠 4. AI 및 강화학습 (AI & Reinforcement Learning)
*   **[⑥ Spot Arm RL Policy 루프 플로우차트 (Spot Arm Policy Action-State Loop)](6_spot_arm_policy_loop.md)**
    *   *내용*: `spot_policy.py` 내의 69차원 관측 정보(Observation) 수집 주기부터 하이브리드 RL 모델(`walking_policy`, `balance_policy`) 추론, 19차원 각 관절 제어 출력(Action) 적용까지의 딥러닝 연동 사이클과 그리퍼 조작 시의 Joint Override를 설명합니다.
*   **[⑦ 객체 인식 및 맵핑 통합 시퀀스 (YOLOv8 Object Detection & Mapping)](7_yolov8_detection_mapping.md)**
    *   *내용*: `yolov8n.pt`를 통해 그리퍼 카메라 이미지에서 사람(`person`)을 탐지하고, 거리 정보 기반으로 `SEARCHING` ➔ `ALIGNING` ➔ `APPROACHING` ➔ `ESCORTING` (안전 대피 유도) ➔ `DONE` 상태로 진행되는 탐지-접근-안내 통합 시퀀스를 도식화합니다.

### 💼 5. 비즈니스 및 서비스 시나리오 (Business & Service Flow)
*   **[⑧ 전체 서비스 시나리오 플로우차트 (End-to-End Service Scenario)](8_end_to_end_service_scenario.md)**
    *   *내용*: 복잡한 ROS 2 코드 이름을 제외하고, 비개발자(관리자, 기획자 등)가 이 프로젝트의 핵심 비즈니스 가치("화재 자동 감지 및 진압, 조난자 자동 수색 및 대피 안내 서비스")를 5초 만에 파악할 수 있도록 서비스 흐름을 그립니다.
