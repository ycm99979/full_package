#!/bin/bash
# ============================================================================
# Motor Ready Position Calibration Script
# ============================================================================
#
# 현재 모터 위치를 "ready" 위치로 파일에 저장하는 스크립트
# (일자로 쭉 펴진 상태)
#
# 사용법:
#   1. 로봇팔을 일자로 쭉 펴진 상태로 수동으로 이동
#   2. ./calibrate_ready.sh 실행
#
# 저장 위치: ~/.ros/frbot_ready_positions.yaml
#
# ============================================================================

CAN_IF=${1:-can0}
CONFIG_DIR="$HOME/.ros"
CONFIG_FILE="$CONFIG_DIR/frbot_ready_positions.yaml"

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║       Motor Ready Position Calibration                       ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  현재 모터 위치를 'ready' 위치로 파일에 저장합니다           ║"
echo "║  (일자로 쭉 펴진 상태)                                       ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "CAN Interface: $CAN_IF"
echo "Config File: $CONFIG_FILE"
echo ""

# CAN 인터페이스 확인
if ! ip link show $CAN_IF &> /dev/null; then
    echo "❌ 오류: $CAN_IF 인터페이스가 없습니다."
    echo "   먼저 SLCAN을 설정하세요: sudo ./scripts/setup_slcan.sh"
    exit 1
fi

# 설정 디렉토리 생성
mkdir -p "$CONFIG_DIR"

# 모터 ID 배열 (1~4)
MOTOR_IDS=(1 2 3 4)

# 조인트 이름 매핑
declare -A JOINT_NAMES
JOINT_NAMES[1]="link2_to_link1"
JOINT_NAMES[2]="link3_to_link2"
JOINT_NAMES[3]="link4_to_link3"
JOINT_NAMES[4]="gripper_to_link4"

echo "========================================"
echo "  현재 모터 위치 읽기 (0x94: Single circle angle)"
echo "========================================"
echo ""

TMPFILE=$(mktemp)

# candump 시작
timeout 3s candump $CAN_IF > $TMPFILE &
CANDUMP_PID=$!
sleep 0.5

# 각 모터의 현재 위치 읽기 (0x94: Read single circle angle - home 기준 상대 위치)
for ID in "${MOTOR_IDS[@]}"; do
    CAN_ID=$(printf "%03X" $((0x140 + ID)))
    cansend $CAN_IF ${CAN_ID}#9400000000000000 2>/dev/null
    sleep 0.2
done

wait $CANDUMP_PID 2>/dev/null

# 위치 데이터 파싱 및 저장
echo "# FRBot Ready Position Configuration" > "$CONFIG_FILE"
echo "# Generated: $(date)" >> "$CONFIG_FILE"
echo "# 일자로 쭉 펴진 상태의 각 조인트 위치 (라디안)" >> "$CONFIG_FILE"
echo "# home(0) 기준 상대 위치" >> "$CONFIG_FILE"
echo "" >> "$CONFIG_FILE"
echo "ready_positions:" >> "$CONFIG_FILE"

for ID in "${MOTOR_IDS[@]}"; do
    CAN_ID=$(printf "%03X" $((0x140 + ID)))
    RESPONSE_ID=$(printf "%03X" $((0x240 + ID)))
    RESPONSE=$(grep -E "$CAN_IF[[:space:]]+$RESPONSE_ID" $TMPFILE | grep "94" | tail -1)
    
    if [ ! -z "$RESPONSE" ]; then
        # 응답 데이터 추출
        # 0x94 응답 형식: 94 00 XX XX YY YY 00 00
        # XX XX = circle angle (0-65535, 0.01도 단위, 0-360도)
        # YY YY = original angle (signed, 0.01도 단위)
        
        # 바이트 추출 (awk로 필드 추출)
        B3=$(echo "$RESPONSE" | awk '{print $6}')  # circle angle low
        B4=$(echo "$RESPONSE" | awk '{print $7}')  # circle angle high
        B5=$(echo "$RESPONSE" | awk '{print $8}')  # original angle low  
        B6=$(echo "$RESPONSE" | awk '{print $9}')  # original angle high
        
        # Original angle (signed 16-bit) - home 기준 상대 위치
        ANGLE_RAW=$(printf "%d" "0x${B6}${B5}" 2>/dev/null)
        
        # signed 16-bit 처리
        if [ $ANGLE_RAW -gt 32767 ]; then
            ANGLE_RAW=$((ANGLE_RAW - 65536))
        fi
        
        # 0.01도 단위를 라디안으로 변환
        ANGLE_DEG=$(echo "scale=2; $ANGLE_RAW / 100" | bc)
        ANGLE_RAD=$(echo "scale=4; $ANGLE_DEG * 3.14159265359 / 180" | bc)
        
        JOINT_NAME="${JOINT_NAMES[$ID]}"
        
        echo "  ✅ 모터 $ID ($JOINT_NAME):"
        echo "     각도: ${ANGLE_DEG}° = ${ANGLE_RAD} rad"
        
        echo "  $JOINT_NAME: $ANGLE_RAD  # Motor $ID: ${ANGLE_DEG} deg" >> "$CONFIG_FILE"
    else
        echo "  ❌ 모터 $ID: 응답 없음"
        JOINT_NAME="${JOINT_NAMES[$ID]}"
        echo "  $JOINT_NAME: 0.0  # Motor $ID: NO RESPONSE" >> "$CONFIG_FILE"
    fi
done

rm -f $TMPFILE

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ✅ Ready 위치 저장 완료!                                     ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║                                                              ║"
echo "║  저장 위치: $CONFIG_FILE"
echo "║                                                              ║"
echo "║  이 파일의 값들이 하드웨어 인터페이스에서 사용됩니다         ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# 저장된 파일 내용 출력
echo "저장된 내용:"
echo "----------------------------------------"
cat "$CONFIG_FILE"
echo "----------------------------------------"
