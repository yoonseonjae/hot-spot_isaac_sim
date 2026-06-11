# ⑤ 로봇 정찰/순찰 임무 시퀀스 다이어그램 (Room Patrol Mission Sequence Diagram)

본 다이어그램은 `robot2_room_patrol.py` 노드가 시작 신호를 받아 정해진 방들을 순찰하는 일련의 자율주행/제자리 회전 시퀀스와, 순찰 도중 YOLOv8 카메라를 통해 조난자를 감지했을 때 순찰 노드 제어권을 가로채고 인명 구조 모드로 급격히 전환되는 비상 흐름을 시퀀스 다이어그램으로 나타냅니다.

```mermaid
sequenceDiagram
    autonumber
    actor User as 관리자 / 시스템
    participant Patrol as robot2_room_patrol 노드
    participant Nav2 as Nav2 Action 서버<br>(/robot2/navigate_to_pose)
    participant Spin as Nav2 Spin Action 서버<br>(/robot2/spin)
    participant Agent as robot2 에이전트<br>(spot_agent.py / Isaac Sim)
    participant YOLO as YOLOv8 비전 분석<br>(yolov8n.pt)

    %% 1. 순찰 활성화
    User->>Patrol: /robot2/start_room_patrol [Empty 토픽 발행]
    Note over Patrol: 순찰 플래그 활성화 (running = True)<br>웨이포인트 인덱스 초기화 (index = 0)

    %% 2. 순찰 루프 시작 (5번방 이동)
    Note over Patrol, Agent: [루프 시작] 1번째 지점 (5번방) 순찰 시작
    Patrol->>Nav2: NavigateToPose 목표 전송 (x=5.039, y=-6.990)
    Nav2-->>Patrol: Goal 수락 및 피드백 전송
    
    rect rgb(240, 248, 255)
        Note over Nav2, Agent: 자율주행 실행
        Nav2->>Agent: cmd_vel 주행 명령 송신
        Agent->>Agent: Isaac Sim 내 물리 이동 및 센서 갱신
        Agent-->>Nav2: 현재 위치 피드백 (Pose Bridge 연동)
    end

    Nav2->>Patrol: Goal 도착 완료 반환 (STATUS_SUCCEEDED)
    
    %% 제자리 회전 액션
    Patrol->>Spin: Spin 목표 전송 (target_yaw = 360도 회전)
    Spin-->>Patrol: Goal 수락
    
    rect rgb(245, 245, 245)
        Note over Spin, Agent: 제자리 회전 실행
        Spin->>Agent: cmd_vel 회전 속도 송신
        Agent->>Agent: 360도 주변 탐색 스핀 동작 수행
    end

    Spin->>Patrol: Spin 완료 반환 (STATUS_SUCCEEDED)
    Note over Patrol: 다음 지점으로 인덱스 증가 (index = 1)

    %% 3. 순찰 루프 2단계 및 비상 중단 흐름 (YOLO 감지)
    Note over Patrol, Agent: 2번째 지점 (1번방) 순찰 시작
    Patrol->>Nav2: NavigateToPose 목표 전송 (x=-1.707, y=-11.104)
    Nav2-->>Patrol: Goal 수락

    loop 실시간 카메라 분석
        Agent->>YOLO: 그리퍼 카메라 RGB 프레임 전송
        YOLO->>YOLO: 객체 검출 추론 실행 (class == 0 : person)
    end

    rect rgb(255, 235, 235)
        Note over Agent, YOLO: [비상 상황 발생] 👤 사람(조난자) 감지 성공!
        YOLO-->>Agent: Person 발견 알림 (거리 및 중심점 cx 획득)
        Note over Agent: 순찰 동작 즉시 가로채기<br>_patrol_active = False 설정
        Agent->>Nav2: Nav2 목표 취소 요청 (Cancel Goal)
        Nav2-->>Patrol: Goal 비정상 취소 반환 (STATUS_CANCELED)
        Note over Patrol: 순찰 노드 대기 상태로 전환 (running = False)
    end

    %% 4. 구조 상태머신 전환
    Note over Agent: 조난자 정렬 및 접근 시퀀스 구동<br>(SEARCHING ➔ ALIGNING ➔ APPROACHING)
    Agent->>Agent: 조난자를 향해 정속 접근 (1.15m 거리 이내)
    Note over Agent: 안전 유도 모드 가동 (ESCORTING)<br>Nav2 목표지를 출구(EXIT_POS)로 강제 재설정
    Agent->>Nav2: NavigateToPose 목표 전송 (x=1.568, y=20.041)
    Note over Agent: 사람이 뒤따라오도록 보행 유도 (Follow)
    Nav2->>Agent: 출구로 자율주행 안내 실행
    Nav2-->>Agent: 출구 도착 성공 반환
    Note over Agent: 조난자 안전 탈출 성공 (DONE)
```

### 📋 시퀀스 세부 동작 설명

1.  **순찰 시작 트리거**:
    *   사용자가 `/robot2/start_room_patrol` Empty 메시지를 발행하면 `Robot2RoomPatrol` 클래스의 콜백 함수 `_on_start`가 호출되면서 시작됩니다.
2.  **동작-회전 연계 제어**:
    *   로봇은 방 번호 순서(`5번방 ➔ 1번방 ➔ 3번방 ➔ 4번방 ➔ 6번방`)대로 목표 지점을 순회합니다.
    *   목표 지점에 도착(액션 성공)하면, 로봇은 방 내부의 사각지대를 수색하기 위해 `/robot2/spin` 액션을 통해 **제자리에서 시계 방향으로 360도 회전(target_yaw = 2 * pi)**을 수행한 후 다음 방으로 이동합니다.
3.  **YOLOv8 비전 분석에 의한 중단 (Interrupt)**:
    *   자율주행 중인 로봇의 카메라 프레임 분석 스레드(`_run_yolo_step`)에서 사람 클래스(`person`)가 임계 확률(`COBOT_YOLO_CONF` = 0.35) 이상으로 감지되면 순찰 시퀀스가 비상 중단됩니다.
    *   `spot_agent.py`는 액션 클라이언트에 직접 취소 명령을 내려 Nav2 주행을 강제 중단시키고, 로봇은 기존 순찰 경로를 이탈하여 조난자 대피 가이드 모드로 돌입합니다.

---

### 🛠️ 주요 트러블슈팅 사례 (Troubleshooting)

1. **Nav2 액션 서버 먹통 및 통신 명령 유실**
   - **문제 상황**: 시뮬레이션 환경의 부하나 네트워크 지연으로 인해 Nav2 액션 서버가 먹통이 되거나 이동 명령 자체가 유실되는 상황이 발생했습니다.
   - **해결책**: 이를 해결하기 위해 `applications/spot_agent.py` 스크립트에 `_nav_goal_retry` 로직을 구현했습니다. 명령이 유실되거나 주행 반응이 없다고 판단되면 시스템이 자동으로 목표 지점을 재전송(Retry)하게 만들어 통신 신뢰성을 완벽히 확보했습니다.

