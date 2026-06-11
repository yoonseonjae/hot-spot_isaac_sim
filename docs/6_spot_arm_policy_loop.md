# ⑥ Spot Arm RL Policy 루프 플로우차트 (Spot Arm Policy Action-State Loop)

본 다이어그램은 `spot_policy.py` 내의 `SpotArmFlatTerrainPolicy` 제어기에서 실행되는 **강화학습(RL) 기반의 보행 및 균형 제어 루프**와 고수준 동작 시의 **관절 값 강제 지정(Joint Override)** 메커니즘을 상세히 도식화합니다.

```mermaid
flowchart TD
    %% 스타일 정의
    classDef obs fill:#e1f5fe,stroke:#0288d1,stroke-width:2px;
    classDef policy fill:#ede7f6,stroke:#5e35b1,stroke-width:2px;
    classDef act fill:#fff3e0,stroke:#ff9800,stroke-width:2px;
    classDef override fill:#ffebee,stroke:#c62828,stroke-width:2px;
    classDef physics fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px;

    %% 1. 관측 정보 수집 단계
    subgraph OBSERVATION["📊 69차원 관측 정보 구성 (_compute_observation)"]
        direction TB
        OB1["선속도 & 각속도 (Base velocities)<br>- 몸체 기준 lin_vel_b (3차원)<br>- 몸체 기준 ang_vel_b (3차원)"]:::obs
        OB2["투영된 중력 벡터 (Projected gravity)<br>- gravity_b (3차원)"]:::obs
        OB3["고수준 속도 명령 (Driving command)<br>- v_x, v_y, w_z (3차원)"]:::obs
        OB4["관절 위치 편차 (Joint position error)<br>- current_joint_pos - default_pos<br>(19차원: 다리 12 + 팔/그리퍼 7)"]:::obs
        OB5["관절 속도 (Joint velocities)<br>- current_joint_vel (19차원)"]:::obs
        OB6["이전 행동 값 (Previous action)<br>- _previous_action (19차원)"]:::obs

        OB1 & OB2 & OB3 & OB4 & OB5 & OB6 --> OBS_VEC["69차원 Observation 벡터 완성"]:::obs
    end

    %% 2. 정책 모델 선택 및 추론 단계
    subgraph INFERENCE["🧠 하이브리드 RL 정책 모델 선택 및 추론"]
        direction TB
        SEL_P{"현재 로봇 상태 판별<br>(balance_timer > 0 혹은<br>extend_arm_mode == True인가?)"}:::policy
        
        P_WALK["walking_policy<br>(spot_arm_policy.pt)<br>- 일반적인 사족보행 모드"]:::policy
        P_BAL["balance_policy<br>(model_10800.pt)<br>- 자세 무너짐 방지/균형 모드"]:::policy
        
        SEL_P -->|No| P_WALK
        SEL_P -->|Yes| P_BAL
        
        OBS_VEC --> SEL_P
        
        P_WALK & P_BAL --> OUT_ACT["19차원 Action 벡터 출력"]:::policy
    end

    %% 3. 제어 명령 스케일링 및 물리 출력
    subgraph ACTUATION["⚙️ 제어값 스케일링 및 오버라이드 판단"]
        direction TB
        SCALE["스케일링 연산<br>target_pos = default_pos + Action * 0.2"]:::act
        
        CHK_FREEZE{"override_leg_freeze<br>활성화 여부?"}:::override
        DO_FREEZE["레그 프리즈 (다리 관절 고정)<br>- 직전 다리 관절 각도로 고정<br>- Stiffness=2000, Damping=100 부스트"]:::override
        
        CHK_ARM{"override_arm_angles<br>수동 지정값 존재?"}:::override
        DO_OVERRIDE["팔/그리퍼 조작 오버라이드 (파지/투척)<br>- 목표 각도 강제 지정<br>- Stiffness=5000, Damping=250 강성 강화"]:::override

        OUT_ACT --> SCALE
        SCALE --> CHK_FREEZE
        
        CHK_FREEZE -->|Yes| DO_FREEZE
        CHK_FREEZE -->|No| CHK_ARM
        
        CHK_ARM -->|Yes| DO_OVERRIDE
        CHK_ARM -->|No| DO_APPLY["최종 관절 목표각 연산 완료"]:::act
        
        DO_FREEZE & DO_OVERRIDE --> DO_APPLY
    end

    %% 4. 물리 월드 피드백
    subgraph PHYSICS["🏗️ 물리 연산 및 상태 피드백 (Isaac Sim)"]
        direction TB
        APP_ACT["ArticulationAction 생성<br>(joint_positions, stiffnesses, dampings)"]:::physics
        ROBOT_PHYS["self.robot.apply_action()<br>➔ Spot Arm 가상 관절 토크 구동"]:::physics
        NEXT_STEP["시뮬레이터 물리 1스텝 전진<br>(physics_dt = 1/200s)"]:::physics
        
        APP_ACT --> ROBOT_PHYS --> NEXT_STEP
    end

    %% Closed Loop 연결
    DO_APPLY --> APP_ACT
    NEXT_STEP -.-> |"로봇 상태 센서 재측정"| OBSERVATION
```

### 📋 강화학습 제어 사이클 설명

1.  **관측(Observation) 벡터 구성**:
    *   로봇 몸체(Base) 프레임의 선속도와 각속도, 중력 투영 벡터, ROS 2 브릿지로부터 전송받은 외부 속도 명령, 19개 관절의 위치 오차 및 속도, 직전 제어기 출력 등 총 **69차원**의 정보로 상태 벡터를 조립하여 정책 신경망의 입력으로 제공합니다.
2.  **하이브리드 정책 신경망 (Hybrid Policy)**:
    *   **보행 정책 (`walking_policy`)**: 로봇의 안정적인 보행을 전담합니다.
    *   **균형 정책 (`balance_policy`)**: 몸체의 기울어짐 각도가 임계치(`TILT_SLOWDOWN_RAD` = 14도)를 넘거나 급격히 넘어지는 조짐이 감지되면 보행 정책 대신 동작하여 전도되지 않도록 제어합니다.
3.  **다리 고정 제어 (`override_leg_freeze`)**:
    *   로봇이 멈춰 서서 물건을 집거나 던질 때, 다리가 흔들려 딥러닝 기반 파지 작업이 실패하는 것을 방지하기 위해 다리 관절의 Stiffness를 2000으로 강화하고 관절 각도를 잠금 처리하여 기저부를 단단하게 고정합니다.
4.  **관절 오버라이드 및 강성 강화 (Grasp & Throw Override)**:
    *   소화기를 잡거나 던지는 순간에는 RL 보행 정책에 의해 제어되던 팔 관절을 임시 해제하고, 기하학적 픽앤플레이스 알고리즘에 의해 계산된 목표 관절 위치(`override_arm_angles` 6개, `override_grip_angle` 1개)를 강제로 적용합니다. 
    *   이때 소화기를 들고 있는 동안 발생하는 물리적 부하에 버티기 위해 팔 관절 강성 제어값(Stiffness/Damping)을 평상시보다 강한 **5000 / 250** 수준으로 급격히 부스팅합니다.
