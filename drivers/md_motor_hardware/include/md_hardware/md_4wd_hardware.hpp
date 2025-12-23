/**
 * ============================================================================
 * MD Motor Driver 4WD Hardware Interface for ros2_control
 * ============================================================================
 * 
 * 4륜 개별 제어용 Hardware Interface
 * 2개의 MD 듀얼채널 모터 드라이버를 사용하여 4개 바퀴를 개별 제어
 * 
 * [하드웨어 구성]
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                    RS-485 Bus (Single Serial Port)                      │
 * │  ┌─────────────────────────────┐  ┌─────────────────────────────┐      │
 * │  │   MD Driver #1 (ID=1)       │  │   MD Driver #2 (ID=2)       │      │
 * │  │  ┌─────────┐ ┌─────────┐   │  │  ┌─────────┐ ┌─────────┐   │      │
 * │  │  │Front    │ │Front    │   │  │  │Rear     │ │Rear     │   │      │
 * │  │  │Left     │ │Right    │   │  │  │Left     │ │Right    │   │      │
 * │  │  │(CH1)    │ │(CH2)    │   │  │  │(CH1)    │ │(CH2)    │   │      │
 * │  │  └─────────┘ └─────────┘   │  │  └─────────┘ └─────────┘   │      │
 * │  └─────────────────────────────┘  └─────────────────────────────┘      │
 * └─────────────────────────────────────────────────────────────────────────┘
 * 
 * [ros2_control 아키텍처]
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        Controller Manager                               │
 * │  ┌──────────────────────────┐  ┌─────────────────────────────────────┐ │
 * │  │  diff_drive_controller   │  │    joint_state_broadcaster          │ │
 * │  │  (wheels_per_side: 2)    │  │                                     │ │
 * │  └──────────┬───────────────┘  └────────────────┬────────────────────┘ │
 * │             │                                   │                      │
 * │             ▼                                   ▼                      │
 * │  ┌──────────────────────────────────────────────────────────────────┐  │
 * │  │              MD4WDHardware (이 클래스)                           │  │
 * │  │  - 4개 조인트: FL, FR, RL, RR                                   │  │
 * │  │  - 2개 모터 드라이버: front_driver(ID=1), rear_driver(ID=2)     │  │
 * │  └──────────────────────────────────────────────────────────────────┘  │
 * └─────────────────────────────────────────────────────────────────────────┘
 * 
 * [URDF 설정 예시]
 * <ros2_control name="md_4wd_system" type="system">
 *   <hardware>
 *     <plugin>md_hardware/MD4WDHardware</plugin>
 *     <param name="port">/dev/ttyMotor</param>
 *     <param name="baudrate">57600</param>
 *     <param name="front_driver_id">1</param>
 *     <param name="rear_driver_id">2</param>
 *   </hardware>
 *   <joint name="front_left_wheel_joint">...</joint>
 *   <joint name="front_right_wheel_joint">...</joint>
 *   <joint name="rear_left_wheel_joint">...</joint>
 *   <joint name="rear_right_wheel_joint">...</joint>
 * </ros2_control>
 * 
 * ============================================================================
 */

#ifndef MD_HARDWARE__MD_4WD_HARDWARE_HPP_
#define MD_HARDWARE__MD_4WD_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <array>

// ros2_control
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Serial communication
#include "serial/serial.h"

namespace md_hardware
{

/* ============================================================================
 * 상수 정의 (MD Motor Protocol)
 * ============================================================================ */

// 통신 상태
constexpr int ON_4WD = 1;
constexpr int OFF_4WD = 0;
constexpr int SUCCESS_4WD = 1;
constexpr int FAIL_4WD = 0;

// PID (Protocol ID) - MD 모터 드라이버 명령
constexpr uint8_t PID_REQ_PID_DATA_4WD = 4;      // 데이터 요청
constexpr uint8_t PID_TQ_OFF_4WD = 5;            // 토크 OFF
constexpr uint8_t PID_COMMAND_4WD = 10;          // 일반 명령
constexpr uint8_t PID_POSI_RESET_4WD = 13;       // 위치 리셋
constexpr uint8_t PID_VEL_CMD_4WD = 130;         // 단일 모터 속도 명령
constexpr uint8_t PID_MAIN_DATA_4WD = 193;       // 메인 데이터 (피드백)
constexpr uint8_t PID_PNT_VEL_CMD_4WD = 207;     // 듀얼 모터 속도 명령 ★

// 패킷 크기
constexpr size_t MAX_PACKET_SIZE_4WD = 26;
constexpr size_t MAX_DATA_SIZE_4WD = 23;

// 데이터 요청 타입
constexpr uint8_t REQUEST_PNT_MAIN_DATA_4WD = 2;

// 바퀴 인덱스
constexpr size_t FRONT_LEFT = 0;
constexpr size_t FRONT_RIGHT = 1;
constexpr size_t REAR_LEFT = 2;
constexpr size_t REAR_RIGHT = 3;
constexpr size_t NUM_WHEELS = 4;

/* ============================================================================
 * 바이트 변환 구조체
 * ============================================================================ */
struct IByte4WD {
    uint8_t low;   // 하위 바이트 (LSB)
    uint8_t high;  // 상위 바이트 (MSB)
};

/* ============================================================================
 * MotorDriver 구조체 - 각 모터 드라이버 상태
 * ============================================================================ */
struct MotorDriver {
    int id;                     // 드라이버 ID (1 또는 2)
    
    // 명령 (write)
    int16_t cmd_left_rpm;       // 왼쪽 채널 RPM 명령
    int16_t cmd_right_rpm;      // 오른쪽 채널 RPM 명령
    
    // 피드백 (read)
    int16_t fb_left_rpm;        // 왼쪽 채널 RPM 피드백
    int16_t fb_right_rpm;       // 오른쪽 채널 RPM 피드백
    int32_t fb_left_position;   // 왼쪽 채널 엔코더 위치
    int32_t fb_right_position;  // 오른쪽 채널 엔코더 위치
    
    // 누적 라디안 (odometry용)
    double left_accumulated_rad;
    double right_accumulated_rad;
    int32_t left_last_tick;
    int32_t right_last_tick;
    
    // 통신 상태
    bool initialized;
    int error_count;
};

/* ============================================================================
 * 통신 상태 구조체
 * ============================================================================ */
struct CommState4WD {
    uint8_t send_buf[MAX_PACKET_SIZE_4WD];
    uint8_t recv_buf[MAX_PACKET_SIZE_4WD];
    uint8_t step;                 // 파싱 상태 머신 단계
    uint8_t packet_num;           // 현재 패킷 위치
    uint8_t checksum;             // 체크섬
    uint8_t max_data_num;         // 예상 데이터 크기
    uint8_t data_num;             // 현재 데이터 수
    bool packet_ok;               // 패킷 수신 완료 플래그
    uint8_t error_count;          // 통신 오류 카운트
    uint8_t current_driver_id;    // 현재 수신 중인 드라이버 ID
};

/* ============================================================================
 * MD4WDHardware 클래스
 * ============================================================================
 * 
 * ros2_control Hardware Interface 구현
 * 2개의 MD 듀얼채널 모터 드라이버로 4륜 개별 제어
 * ============================================================================ */
class MD4WDHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MD4WDHardware)

    /* ────────────────────────────────────────────────────────────────────────
     * Lifecycle 콜백 함수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    /* ────────────────────────────────────────────────────────────────────────
     * Interface Export 함수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /* ────────────────────────────────────────────────────────────────────────
     * 제어 루프 함수들 (매 주기 호출)
     * ──────────────────────────────────────────────────────────────────────── */
    
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    /* ────────────────────────────────────────────────────────────────────────
     * 시리얼 통신 함수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    int initSerial();
    int sendVelocityCommand(MotorDriver& driver);
    int requestFeedback(MotorDriver& driver);
    int receiveData();
    int analyzeReceivedData(uint8_t* buffer, size_t size);
    void processReceivedData(uint8_t driver_id);

    /* ────────────────────────────────────────────────────────────────────────
     * 단위 변환 함수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    int16_t radPerSecToRpm(double rad_per_sec);
    double rpmToRadPerSec(int16_t rpm);
    double tickToRad(int32_t tick);

    /* ────────────────────────────────────────────────────────────────────────
     * 바이트 변환 유틸리티
     * ──────────────────────────────────────────────────────────────────────── */
    
    IByte4WD short2Byte(int16_t value);
    int16_t byte2Short(uint8_t low, uint8_t high);
    int32_t byte2Long(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4);

    /* ────────────────────────────────────────────────────────────────────────
     * 디버그/에러 로깅 함수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    void printSerialStatus();
    void printDriverStatus(const MotorDriver& driver, const std::string& name);
    void printAllWheelStatus();
    void printErrorReport(const std::string& function_name, 
                          const std::string& error_msg,
                          int error_code = 0);

    /* ────────────────────────────────────────────────────────────────────────
     * 멤버 변수들
     * ──────────────────────────────────────────────────────────────────────── */
    
    // 시리얼 통신 객체
    serial::Serial serial_;

    // ─────────────────────────────────────────────────────────────────────
    // 통신 파라미터 (URDF에서 로드)
    // ─────────────────────────────────────────────────────────────────────
    std::string port_;                // 시리얼 포트 (예: "/dev/ttyMotor")
    int baudrate_;                    // 통신 속도 (예: 57600)
    int id_mdui_;                     // MDUI ID (184)
    int id_mdt_;                      // MDT ID (183)

    // ─────────────────────────────────────────────────────────────────────
    // 모터 드라이버 (2개)
    // ─────────────────────────────────────────────────────────────────────
    MotorDriver front_driver_;        // 전방 드라이버 (ID=1): FL, FR
    MotorDriver rear_driver_;         // 후방 드라이버 (ID=2): RL, RR

    // ─────────────────────────────────────────────────────────────────────
    // 로봇 파라미터 (URDF에서 로드)
    // ─────────────────────────────────────────────────────────────────────
    double wheel_radius_;             // 바퀴 반경 (m)
    double wheel_separation_;         // 좌우 바퀴 간격 (m)
    double wheelbase_;                // 전후 바퀴 간격 (m)
    int gear_ratio_;                  // 기어비
    int poles_;                       // 모터 극 수

    // ─────────────────────────────────────────────────────────────────────
    // 엔코더 변환 계수
    // ─────────────────────────────────────────────────────────────────────
    double ppr_;                      // Pulses Per Revolution
    double tick_to_rad_;              // 틱 → 라디안 변환 계수

    // ─────────────────────────────────────────────────────────────────────
    // 통신 상태
    // ─────────────────────────────────────────────────────────────────────
    CommState4WD comm_;

    // ─────────────────────────────────────────────────────────────────────
    // Hardware Interface 버퍼 (ros2_control 연동)
    // ─────────────────────────────────────────────────────────────────────
    
    // 명령 버퍼 (Controller → Hardware)
    // [0]=FL, [1]=FR, [2]=RL, [3]=RR velocity (rad/s)
    std::array<double, NUM_WHEELS> hw_commands_;

    // 상태 버퍼 (Hardware → Controller)
    std::array<double, NUM_WHEELS> hw_positions_;   // position (rad)
    std::array<double, NUM_WHEELS> hw_velocities_;  // velocity (rad/s)

    // 조인트 이름 (URDF에서 로드)
    std::vector<std::string> joint_names_;

    // 스레드 안전을 위한 뮤텍스
    std::mutex mutex_;
    
    // 로거 이름
    static constexpr const char* LOGGER_NAME = "MD4WDHardware";
};

}  // namespace md_hardware

#endif  // MD_HARDWARE__MD_4WD_HARDWARE_HPP_
