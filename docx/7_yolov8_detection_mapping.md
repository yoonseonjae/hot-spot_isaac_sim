# ⑦ 객체 인식 및 맵핑 통합 시퀀스 (YOLOv8 Object Detection & Mapping Flow)

본 다이어그램은 `yolov8n.pt` 딥러닝 모델을 탑재한 순찰 로봇(`robot2`)이 이동 중 조난자(`person`)를 발견하고, 시각 정렬 및 센서 융합을 통해 거리를 측정하여 최종적으로 대피 안전지대까지 안전하게 호송(Escort)하기 위한 **YOLOv8 인명 탐지 상태머신**과 **내비게이션 맵핑 통합 프로세스**를 보여줍니다.

```mermaid
stateDiagram-v2
    %% 스타일 정의
    classDef state_style fill:#f9f9f9,stroke:#333,stroke-width:1px;
    classDef active_style fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px;
    classDef alert_style fill:#ffebee,stroke:#c62828,stroke-width:2px;

    [*] --> SEARCHING : 로봇2 시작 (순찰 상태)
    
    state SEARCHING {
        [*] --> RoomPatrol : 자율 순찰 주행 중
        RoomPatrol --> CaptureFrame : 그리퍼 카메라 이미지 획득
        CaptureFrame --> YOLO_Inference : YOLOv8 추론 연산 (yolov8n.pt)
        YOLO_Inference --> PersonDetected : 사람 검출? (class == 0 && conf > 0.35)
        
        PersonDetected --> RoomPatrol : No (순찰 지속)
    }
    
    %% 비상 전환
    SEARCHING --> ALIGNING : Yes (사람 발견!)
    note left of ALIGNING
        순찰 루프 즉시 종료
        _patrol_active = False
        Nav2 목표 취소 (Cancel Goal)
    end

    state ALIGNING {
        [*] --> ComputeOffset : Bounding Box 중심점 cx와 이미지 중심 비교
        ComputeOffset --> RotateRobot : P제어 기반 회전 속도 wz 인가
        RotateRobot --> CheckAligned : 정렬각 허용 오차 이내인가?<br>(heading_error < 0.10 rad)
        CheckAligned --> RotateRobot : No (정렬 유지)
    }

    ALIGNING --> APPROACHING : Yes (정렬 완료)

    state APPROACHING {
        [*] --> GetDepth : 거리 측정<br>- depth_np[cy, cx] 버퍼 값 쿼리<br>- 실패 시 Camera Raycast 백업 작동
        GetDepth --> DriveForward : 직진 속도 vx 인가하여 조난자 방향 접근
        DriveForward --> DistanceCheck : 1.15m 이내 근접 & 0.8초 지속?<br>(또는 8.0초 타임아웃 도달?)
        DistanceCheck --> DriveForward : No (계속 접근)
    }

    APPROACHING --> ESCORTING : Yes (접근 성공)
    note left of ESCORTING
        대피 유도 모드 진입
        - 조난자 충돌체 비활성화 (collision = False)
        - Nav2 Costmap 강제 초기화 (Clear Costmaps)
        - Nav2 목표지를 출구(EXIT_POS)로 강제 설정
    end

    state ESCORTING {
        [*] --> ExitNav2 : 출구(x=1.568, y=20.041)로 자율주행 실행
        ExitNav2 --> UpdatePersonPose : 사람을 로봇 뒤로 이동 유도 (Follow)<br>- 로봇 뒤 2.4m 거리로 좌표 강제 변환<br>- Person2 바운딩 박스(붉은색 격자) 동적 갱신
        UpdatePersonPose --> CheckExitArrival : 출구 거리 < 0.8m 도달?
        CheckExitArrival --> ExitNav2 : No
    }

    ESCORTING --> DONE : Yes (출구 도착)
    
    state DONE {
        [*] --> CompleteRescue : 주행 속도 정지 (Zero Velocity)<br>탈출 완료 콘솔 로깅 및 시뮬레이션 일시정지
    }

    DONE --> [*]
```

### 📋 통합 프로세스 기술 설명

1.  **순찰 중 프레임 분석 (`SEARCHING`)**:
    *   로봇2의 카메라 센서 루프는 실시간으로 RGB 프레임을 `yolov8n.pt` 모델의 입력으로 제공하여 사람(`person` 클래스)이 존재하는지 모니터링합니다.
    *   사람이 탐지되면 즉시 `_patrol_active` 플래그를 해제하고 Nav2의 순찰 목적지 액션을 중단하여 제어권을 YOLO 상태머신으로 일임합니다.
2.  **화면 중심 정렬 (`ALIGNING`)**:
    *   감지된 바운딩 박스의 중심좌표(`cx`)와 카메라의 센터 좌표를 정렬하기 위해, 로봇은 선속도를 0으로 유지한 채 각속도(`wz`)만으로 제자리 회전하여 정렬 상태를 달성합니다.
3.  **거리 센서 융합 접근 (`APPROACHING`)**:
    *   정렬된 조난자의 거리 값을 계산하기 위해, 카메라의 Depth Plane 센서 버퍼에서 정렬점 픽셀의 깊이 정보(`depth_np[cy, cx]`)를 조회합니다. 버퍼 수집 오류가 발생할 경우를 대비하여 물리적 레이캐스트(`_camera_raycast_person_detection`) 값을 백업으로 융합합니다.
    *   조난자 방향으로 이동하여 **안전 거리 1.15m**에 접근하면 구조 안내 단계로 이동합니다.
4.  **내비게이션 맵 복구 및 유도 (`ESCORTING`)**:
    *   **충돌체 차단**: 조난자(`Person2`)가 로봇 바로 뒤에서 따라오면, ROS 2의 라이다 스캔 센서가 사람을 장애물로 판단하여 Nav2 Costmap에 장애물 영역을 팽창시킵니다. 이로 인해 로봇의 탈출 경로가 가로막혀 제자리에서 회전만 하게 되는 병목(Stuck) 현상이 생깁니다.
    *   **해결책**: 이를 예방하기 위해 조난자의 물리 충돌체를 일시 비활성화(`_set_person2_collision_enabled(False)`)하고, ROS 2 Costmap을 완전히 비운(`_clear_robot2_costmaps`) 뒤 출구(`EXIT_POS`)로 자율주행 경로를 다시 설계합니다.
    *   **조난자 트래킹**: 이동하는 동안 조난자는 로봇의 2.4m 후방 위치로 계속 텔레포트(`_follow_person2_behind_robot`)하여 로봇이 출구로 조난자를 호송해 나가는 비주얼을 시뮬레이터 상에 완성하고, 붉은색 검출 마커 바운딩 박스를 씌워 실시간 맵핑 상황을 Dashboard에 나타냅니다.
