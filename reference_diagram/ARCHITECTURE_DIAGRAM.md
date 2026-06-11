# 로봇 시스템 통신 아키텍처 (ROS2 Communication Diagram)

제공해주신 레퍼런스 이미지를 바탕으로, 현재 프로젝트(인생두컷)의 전체 통신 아키텍처를 표현한 다이어그램입니다. 메인 컨트롤러인 `robot_control_07.py`와 `robot_control_node_05.py`가 2개의 핵심 마더보드(Coordinator) 역할을 수행하는 구조를 명확히 담았습니다.

```mermaid
flowchart LR
    %% 통신 방식 범례
    %% 주황색: Topic, 초록색: Service, 파란색: Action, 보라색: REST API
    classDef default fill:#f9f9f9,stroke:#333,stroke-width:1px;
    classDef coord fill:#e1f5fe,stroke:#0277bd,stroke-width:2px;

    subgraph VOICE["음성 사용자 인터페이스 (VOICE)"]
        direction LR
        STT["STT\n(마이크 입력)"] --> GPT["GPT-4o\n(자연어 처리)"]
    end

    subgraph VISION["비전 센서 (REALSENSE & WEBCAM)"]
        direction TB
        RS_C["컬러 영상 스트림"]
        RS_D["깊이 영상 스트림"]
        WEB_C["상단뷰 영상 스트림"]
    end

    subgraph IMG_PROC["이미지 프로세서 (AI)"]
        direction TB
        YOLO["YOLOv8 + 좌표 변환\n(소품 3D 좌표 추출)"]
        GES["제스처 분류기\n(방향 및 촬영 제어)"]
        SAFE["안전 감시 모니터\n(안전구역 침범 감지)"]
    end

    subgraph CLOUD["클라우드 / DB"]
        direction TB
        FB["Firebase RTDB\n(상태 동기화)"]
    end

    subgraph CTRL["ROS2 메인 컨트롤러"]
        direction TB
        TC1["작업 코디네이터 1\n[픽앤플레이스]\n(robot_control_07)"]:::coord
        TC2["작업 코디네이터 2\n[제스처 제어]\n(robot_control_05)"]:::coord
        MVR["모션 엔진 & 그리퍼 로직\n(DSR API 래퍼)"]
        STAT["시스템 상태 및 안전\n(상태 머신)"]
        
        TC1 --> MVR
        TC2 --> MVR
        STAT --> TC1
        STAT --> TC2
    end

    subgraph HW["하드웨어 (로봇/그리퍼)"]
        direction TB
        ARM["M0609 로봇 팔"]
        GRP["RG2 그리퍼"]
    end

    %% 내부 연결 (비전 -> AI)
    RS_C --> YOLO
    RS_D --> YOLO
    RS_C --> GES
    WEB_C --> SAFE

    %% 컴포넌트 간 외부 통신 (라벨에 통신 타입 명시)
    GPT -- "사용자 명령 [REST API]" --> FB
    FB -- "상태 변경 트리거 [REST API]" --> TC1
    
    YOLO -- "목표 3D 좌표 [Service]" --> TC1
    GES -- "이동/방향 명령 [Topic]" --> TC2
    SAFE -- "안전 경고 [Topic]" --> STAT
    
    TC1 -- "작업 완료 알림 [Topic]" --> TC2
    
    MVR -- "모션 실행 [Action/Service]" --> ARM
    MVR -- "그리퍼 제어 [Service]" --> GRP
    
    ARM -- "로봇 현재 상태 [Topic]" --> STAT

    %% 스타일 적용 팁:
    %% - Topic: 주로 스트림성 데이터 (영상, 제스처, 상태)
    %% - Service: 즉각적인 요청/응답 (3D 좌표 계산, 그리퍼 제어)
    %% - REST API: 인터넷을 통한 상태 동기화 (Firebase)
```
