# ⑧ 전체 서비스 시나리오 플로우차트 (End-to-End Service Scenario Flowchart)

본 다이어그램은 복잡한 ROS 2 토픽이나 코드 변수명을 제외하고, 비개발자(기획자, 채용 담당자, 클라이언트 등)가 프로젝트의 실질적인 비즈니스 가치와 서비스 시나리오("화재 감지-자동 진압-조난자 수색-안전 대피 유도-통합 관제")를 직관적으로 파악할 수 있도록 서비스 흐름을 나타낸 흐름도입니다.

```mermaid
flowchart TD
    %% 스타일 정의
    classDef actor fill:#eceff1,stroke:#607d8b,stroke-width:2px;
    classDef event fill:#ffebee,stroke:#c62828,stroke-width:2px;
    classDef robot1 fill:#e3f2fd,stroke:#1e88e5,stroke-width:2px;
    classDef robot2 fill:#fff3e0,stroke:#fb8c00,stroke-width:2px;
    classDef dash fill:#f1f8e9,stroke:#558b2f,stroke-width:2px;

    %% 1. 시작 및 비상상황 발생
    USER["👤 시스템 관리자<br>(작전 시작 명령)"]:::actor
    FIRE_EVENT["🔥 건물 내부 화재 발생<br>(2번방 발화 및 대형 연기 확산)"]:::event
    
    USER --> FIRE_EVENT

    %% 2. 중앙 관제 대시보드
    subgraph DASHBOARD["🖥️ 중앙 재난 대응 관제 대시보드 (Dashboard)"]
        MONITOR["실시간 로봇 좌표 추적 및<br>센서 정보 수집"]:::dash
        FIRE_ALERT["화재 센서 감지 경보<br>(경보 알람 발생)"]:::dash
        STATUS_REP["최종 화재 진압 및<br>인명 대피 완료 보고서 표출"]:::dash
        
        MONITOR --> FIRE_ALERT
    end
    
    FIRE_EVENT --> FIRE_ALERT

    %% 3. 로봇 1: 화재 진압 임무
    subgraph MISSION_R1["🦾 로봇 1: 화재 자동 진압 작전"]
        direction TB
        R1_NAV_EXT["1. 화재 경보 수신 후<br>소화기 보관소로 자율 이동"]:::robot1
        R1_GRASP["2. 로봇 팔 정밀 조작을 통한<br>소화기 물리 파지 (Grasp)"]:::robot1
        R1_NAV_FIRE["3. 소화기를 파지한 상태로<br>화재 발생 구역 신속 이동"]:::robot1
        R1_THROW["4. 발화점을 겨냥하여<br>소화기 투척 및 가스 분출"]:::robot1
        R1_EXTINGUISH["5. 불길 완전 진압 완료"]:::robot1
        R1_ESCAPE["6. 집 밖 안전 대피소로 복귀"]:::robot1

        R1_NAV_EXT --> R1_GRASP --> R1_NAV_FIRE --> R1_THROW --> R1_EXTINGUISH --> R1_ESCAPE
    end

    %% 4. 로봇 2: 순찰 및 인명 구조 임무
    subgraph MISSION_R2["🐕 로봇 2: 자율 순찰 및 인명 구조 작전"]
        direction TB
        R2_PATROL["1. 건물 내 모든 취약 격실<br>순환 순찰 (수색 작전 수행)"]:::robot2
        R2_YOLO["2. 지능형 비전 카메라 분석으로<br>실시간 조난자 감지"]:::robot2
        R2_APPROACH["3. 안심 대피 유도를 위해<br>조난자에게 조심스럽게 접근"]:::robot2
        R2_ESCORT["4. 호송 유도 모드 활성화<br>(로봇이 인도하고 조난자가 추적)"]:::robot2
        R2_ESCAPE["5. 조난자를 건물 외부<br>안전 대피소로 안전 탈출 완료"]:::robot2

        R2_PATROL --> R2_YOLO --> R2_APPROACH --> R2_ESCORT --> R2_ESCAPE
    end

    %% 임무 연계 관계
    FIRE_ALERT -->|"진압 특명 하사"| MISSION_R1
    FIRE_ALERT -->|"수색/인명구조 특명 하사"| MISSION_R2

    %% 피드백 및 결과 보고
    R1_ESCAPE & R2_ESCAPE -->|"동시 대피 완료 상태 전송"| MONITOR
    MONITOR --> STATUS_REP
    STATUS_REP -->|"재난 상황 종료 및 결과 리포트"| USER
```

### 💼 비즈니스 시나리오 가치 흐름

1.  **재난 상황 감지 (Dashboard)**:
    *   중앙 관제소는 평시 상황에서 건물 내부에 배치된 두 대의 로봇 위치를 3D 지도로 실시간 모니터링합니다. 화재 센서가 발화 온도에 도달하면 대시보드에 즉시 시각 및 청각 경보가 울리고 각 로봇에게 비상 제어 명령을 전달합니다.
2.  **화재 조기 진압 성공 (로봇 1)**:
    *   로봇 1은 화재 현장으로 직접 뛰어들기 전, 현장에 배치된 소화기를 스스로 찾아가 로봇 팔을 이용해 단단히 움켜쥡니다. 소화기를 파지한 로봇 1은 위험한 화재 영역으로 진입하여 적절한 거리에서 소화기를 정확하게 투척하여 가스 방출을 유도, 대형 화재로 번지는 것을 물리적으로 조기 진압하고 대피소로 복귀합니다.
3.  **조난자 무사 구출 (로봇 2)**:
    *   화재로 인한 연기와 장애물이 늘어나는 최악의 상황 속에서, 로봇 2는 격실을 차례로 자율 수색합니다. 카메라 영상을 기반으로 숨어있는 조난자를 식별해 낸 로봇 2는 조난자가 안심하고 따라올 수 있도록 천천히 다가가 출구 방향으로 비상 유도 자율주행을 실행합니다. 조난자를 안전지대까지 가이드하여 소중한 생명을 무사히 구출합니다.
4.  **관제 종결 및 보고**:
    *   대시보드는 소화 성공 정보와 조난자의 대피 완료 이벤트를 자동으로 판정 및 수집하여 관리자에게 "재난 통제 완료 및 인명 구조 100% 완료" 보고서를 출력함으로써 재난 상황을 평화롭게 종료합니다.
