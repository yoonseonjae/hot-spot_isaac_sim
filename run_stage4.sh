#!/bin/bash
# ============================================================
# Stage 4 실행 스크립트
# 확인 항목:
#   - 맵에 Person1, Person2 스폰되는지
#   - robot2 그리퍼 카메라 창 뜨는지 ("robot2 Gripper View")
#   - robot2가 사람을 발견하면 ALIGNING → APPROACHING → ARRIVED
#   - robot1 소화기 Grasp(G키)/투척(Q키) 여전히 동작
#   - 화재 10초 점화, 15초 확산
# ============================================================
set -e
cd "$(dirname "$0")"

export COBOT_PERSON_ALIGN_TOLERANCE="${COBOT_PERSON_ALIGN_TOLERANCE:-0.10}"
export COBOT_PERSON_APPROACH_DISTANCE="${COBOT_PERSON_APPROACH_DISTANCE:-1.15}"
export COBOT_PERSON_APPROACH_MIN_TIME="${COBOT_PERSON_APPROACH_MIN_TIME:-0.8}"
export COBOT_PERSON_APPROACH_TIMEOUT="${COBOT_PERSON_APPROACH_TIMEOUT:-8.0}"
export COBOT_PERSON_FOLLOW_DISTANCE="${COBOT_PERSON_FOLLOW_DISTANCE:-2.4}"

echo "============================================"
echo " Stage 4: YOLO 인명탐지 + 사람 스폰"
echo "============================================"
echo ""
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh applications/main_simulation.py
