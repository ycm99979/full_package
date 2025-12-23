/**
 * ============================================================================
 * MD Motor Driver 4WD Hardware Interface Implementation
 * ============================================================================
 * 
 * 4륜 개별 제어용 Hardware Interface 구현
 * 2개의 MD 듀얼채널 모터 드라이버를 사용하여 4개 바퀴를 개별 제어
 * 
 * [모터 드라이버 매핑]
 * - Front Driver (ID=1): CH1=Front Left, CH2=Front Right
 * - Rear Driver (ID=2):  CH1=Rear Left,  CH2=Rear Right
 * 
 * ============================================================================
 */

#include "md_hardware/md_4wd_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <errno.h>
#include <cstring>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace md_hardware
{

/* ============================================================================
 * Lifecycle 콜백 함수들
 * ============================================================================ */

hardware_interface::CallbackReturn MD4WDHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║      MD Motor 4WD Hardware Interface Initializing            ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║              (4-Wheel Individual Control)                    ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");

    // 부모 클래스 초기화
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        printErrorReport("on_init", "Parent class initialization failed", 0);
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[INIT] Loading parameters from URDF...");

    // ─────────────────────────────────────────────────────────────────────
    // URDF <ros2_control> 섹션에서 파라미터 로드
    // ─────────────────────────────────────────────────────────────────────
    
    // 시리얼 통신 파라미터
    port_ = info_.hardware_parameters.count("port") > 0 
            ? info_.hardware_parameters.at("port") : "/dev/ttyMotor";
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] port = %s", port_.c_str());
    
    baudrate_ = info_.hardware_parameters.count("baudrate") > 0
                ? std::stoi(info_.hardware_parameters.at("baudrate")) : 57600;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] baudrate = %d", baudrate_);
    
    // 모터 드라이버 ID
    front_driver_.id = info_.hardware_parameters.count("front_driver_id") > 0
                       ? std::stoi(info_.hardware_parameters.at("front_driver_id")) : 1;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] front_driver_id = %d", front_driver_.id);
    
    rear_driver_.id = info_.hardware_parameters.count("rear_driver_id") > 0
                      ? std::stoi(info_.hardware_parameters.at("rear_driver_id")) : 2;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] rear_driver_id = %d", rear_driver_.id);
    
    id_mdui_ = info_.hardware_parameters.count("id_mdui") > 0
               ? std::stoi(info_.hardware_parameters.at("id_mdui")) : 184;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] id_mdui = %d", id_mdui_);
    
    id_mdt_ = info_.hardware_parameters.count("id_mdt") > 0
              ? std::stoi(info_.hardware_parameters.at("id_mdt")) : 183;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] id_mdt = %d", id_mdt_);

    // 로봇 파라미터
    wheel_radius_ = info_.hardware_parameters.count("wheel_radius") > 0
                    ? std::stod(info_.hardware_parameters.at("wheel_radius")) : 0.05;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] wheel_radius = %.4f m", wheel_radius_);
    
    wheel_separation_ = info_.hardware_parameters.count("wheel_separation") > 0
                        ? std::stod(info_.hardware_parameters.at("wheel_separation")) : 0.3;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] wheel_separation = %.4f m", wheel_separation_);
    
    wheelbase_ = info_.hardware_parameters.count("wheelbase") > 0
                 ? std::stod(info_.hardware_parameters.at("wheelbase")) : 0.3;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] wheelbase = %.4f m", wheelbase_);
    
    gear_ratio_ = info_.hardware_parameters.count("gear_ratio") > 0
                  ? std::stoi(info_.hardware_parameters.at("gear_ratio")) : 15;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] gear_ratio = %d", gear_ratio_);
    
    poles_ = info_.hardware_parameters.count("poles") > 0
             ? std::stoi(info_.hardware_parameters.at("poles")) : 10;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[PARAM] poles = %d", poles_);

    // ─────────────────────────────────────────────────────────────────────
    // 엔코더 변환 계수 계산
    // ─────────────────────────────────────────────────────────────────────
    ppr_ = poles_ * 3.0 * gear_ratio_;
    tick_to_rad_ = (2.0 * M_PI) / ppr_;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CALC] PPR = %.1f (poles=%d × 3 × gear_ratio=%d)", 
                ppr_, poles_, gear_ratio_);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CALC] Tick2Rad = %.6f rad/tick", tick_to_rad_);

    // ─────────────────────────────────────────────────────────────────────
    // 조인트 검증 (4개의 휠 조인트 필요)
    // ─────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[INIT] Checking joints...");
    
    if (info_.joints.size() != NUM_WHEELS)
    {
        printErrorReport("on_init", 
            "Expected 4 joints but got " + std::to_string(info_.joints.size()), 0);
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "[ERROR] URDF must define exactly 4 joints for 4WD control!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 조인트 이름 저장 (순서: FL, FR, RL, RR)
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[INIT] Found %zu joints:", info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        joint_names_.push_back(info_.joints[i].name);
        std::string wheel_name;
        switch(i) {
            case FRONT_LEFT:  wheel_name = "Front Left";  break;
            case FRONT_RIGHT: wheel_name = "Front Right"; break;
            case REAR_LEFT:   wheel_name = "Rear Left";   break;
            case REAR_RIGHT:  wheel_name = "Rear Right";  break;
        }
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "  [%zu] %s: %s", i, wheel_name.c_str(), info_.joints[i].name.c_str());
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "       - Command interfaces: %zu", info_.joints[i].command_interfaces.size());
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "       - State interfaces: %zu", info_.joints[i].state_interfaces.size());
    }

    // ─────────────────────────────────────────────────────────────────────
    // 버퍼 초기화
    // ─────────────────────────────────────────────────────────────────────
    hw_commands_.fill(0.0);
    hw_positions_.fill(0.0);
    hw_velocities_.fill(0.0);

    // 드라이버 초기화
    auto initDriver = [](MotorDriver& driver) {
        driver.cmd_left_rpm = 0;
        driver.cmd_right_rpm = 0;
        driver.fb_left_rpm = 0;
        driver.fb_right_rpm = 0;
        driver.fb_left_position = 0;
        driver.fb_right_position = 0;
        driver.left_accumulated_rad = 0;
        driver.right_accumulated_rad = 0;
        driver.left_last_tick = 0;
        driver.right_last_tick = 0;
        driver.initialized = false;
        driver.error_count = 0;
    };
    
    initDriver(front_driver_);
    initDriver(rear_driver_);

    // 통신 상태 초기화
    comm_.step = 0;
    comm_.packet_num = 0;
    comm_.checksum = 0;
    comm_.packet_ok = false;
    comm_.error_count = 0;
    comm_.current_driver_id = 0;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║         ✓ 4WD Initialization Complete!                       ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD4WDHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║              Configuring Serial Connection                   ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CONFIG] Port: %s, Baudrate: %d", 
                port_.c_str(), baudrate_);

    if (initSerial() != SUCCESS_4WD)
    {
        printErrorReport("on_configure", "Failed to open serial port: " + port_, errno);
        return hardware_interface::CallbackReturn::ERROR;
    }

    printSerialStatus();
    
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "[CONFIG] ✓ Serial port configured successfully!");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD4WDHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║                  Activating 4WD Motors                       ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");

    if (!serial_.isOpen())
    {
        printErrorReport("on_activate", "Serial port is not open!", 0);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 초기 명령 0으로 설정
    hw_commands_.fill(0.0);
    
    front_driver_.initialized = true;
    rear_driver_.initialized = true;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[ACTIVATE] ✓ All 4 motors activated!");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[ACTIVATE] Front Driver ID=%d (FL, FR)", front_driver_.id);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[ACTIVATE] Rear Driver ID=%d (RL, RR)", rear_driver_.id);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[ACTIVATE] Ready to receive commands on /cmd_vel");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD4WDHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[DEACTIVATE] Stopping all motors...");

    if (serial_.isOpen())
    {
        // 전방 드라이버 정지
        front_driver_.cmd_left_rpm = 0;
        front_driver_.cmd_right_rpm = 0;
        sendVelocityCommand(front_driver_);
        
        // 후방 드라이버 정지
        rear_driver_.cmd_left_rpm = 0;
        rear_driver_.cmd_right_rpm = 0;
        sendVelocityCommand(rear_driver_);
        
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[DEACTIVATE] Sent stop command to all drivers");
    }

    front_driver_.initialized = false;
    rear_driver_.initialized = false;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[DEACTIVATE] ✓ All motors deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD4WDHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CLEANUP] Closing serial port...");

    if (serial_.isOpen())
    {
        serial_.close();
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CLEANUP] Serial port closed.");
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[CLEANUP] ✓ Cleanup complete!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/* ============================================================================
 * Interface Export 함수들
 * ============================================================================ */

std::vector<hardware_interface::StateInterface>
MD4WDHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < NUM_WHEELS; ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                "[EXPORT] Exported %zu state interfaces (4 joints × 2 states)", 
                state_interfaces.size());

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MD4WDHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < NUM_WHEELS; ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                "[EXPORT] Exported %zu command interfaces (4 joints × 1 command)", 
                command_interfaces.size());

    return command_interfaces;
}

/* ============================================================================
 * 제어 루프 함수들
 * ============================================================================ */

hardware_interface::return_type MD4WDHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 시리얼에서 데이터 수신
    receiveData();

    // Front Driver: FL(left), FR(right)
    // 엔코더 틱 → 라디안 변환
    int32_t fl_diff = front_driver_.fb_left_position - front_driver_.left_last_tick;
    front_driver_.left_accumulated_rad += fl_diff * tick_to_rad_;
    front_driver_.left_last_tick = front_driver_.fb_left_position;
    hw_positions_[FRONT_LEFT] = front_driver_.left_accumulated_rad;
    hw_velocities_[FRONT_LEFT] = rpmToRadPerSec(front_driver_.fb_left_rpm);

    int32_t fr_diff = front_driver_.fb_right_position - front_driver_.right_last_tick;
    front_driver_.right_accumulated_rad += fr_diff * tick_to_rad_;
    front_driver_.right_last_tick = front_driver_.fb_right_position;
    hw_positions_[FRONT_RIGHT] = front_driver_.right_accumulated_rad;
    hw_velocities_[FRONT_RIGHT] = rpmToRadPerSec(front_driver_.fb_right_rpm);

    // Rear Driver: RL(left), RR(right)
    int32_t rl_diff = rear_driver_.fb_left_position - rear_driver_.left_last_tick;
    rear_driver_.left_accumulated_rad += rl_diff * tick_to_rad_;
    rear_driver_.left_last_tick = rear_driver_.fb_left_position;
    hw_positions_[REAR_LEFT] = rear_driver_.left_accumulated_rad;
    hw_velocities_[REAR_LEFT] = rpmToRadPerSec(rear_driver_.fb_left_rpm);

    int32_t rr_diff = rear_driver_.fb_right_position - rear_driver_.right_last_tick;
    rear_driver_.right_accumulated_rad += rr_diff * tick_to_rad_;
    rear_driver_.right_last_tick = rear_driver_.fb_right_position;
    hw_positions_[REAR_RIGHT] = rear_driver_.right_accumulated_rad;
    hw_velocities_[REAR_RIGHT] = rpmToRadPerSec(rear_driver_.fb_right_rpm);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MD4WDHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // rad/s → RPM 변환 및 명령 설정
    // Front Driver: FL(left), FR(right)
    front_driver_.cmd_left_rpm = radPerSecToRpm(hw_commands_[FRONT_LEFT]);
    front_driver_.cmd_right_rpm = radPerSecToRpm(hw_commands_[FRONT_RIGHT]);
    sendVelocityCommand(front_driver_);

    // Rear Driver: RL(left), RR(right)
    rear_driver_.cmd_left_rpm = radPerSecToRpm(hw_commands_[REAR_LEFT]);
    rear_driver_.cmd_right_rpm = radPerSecToRpm(hw_commands_[REAR_RIGHT]);
    sendVelocityCommand(rear_driver_);

    return hardware_interface::return_type::OK;
}

/* ============================================================================
 * 시리얼 통신 함수들
 * ============================================================================ */

int MD4WDHardware::initSerial()
{
    try
    {
        serial_.setPort(port_);
        serial_.setBaudrate(baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        serial_.setTimeout(timeout);
        
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] Opening port %s...", port_.c_str());
        serial_.open();

        if (serial_.isOpen())
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] ✓ Port opened successfully!");
            return SUCCESS_4WD;
        }
    }
    catch (const serial::IOException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] ✗ IOException: %s", e.what());
        printErrorReport("initSerial", std::string("IOException: ") + e.what(), errno);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "[SERIAL] ✗ Exception: %s", e.what());
        printErrorReport("initSerial", std::string("Exception: ") + e.what(), errno);
    }

    return FAIL_4WD;
}

/**
 * sendVelocityCommand - 듀얼 모터 속도 명령 전송
 * 
 * [PID_PNT_VEL_CMD (207) 패킷 구조]
 * [RMID][TMID][ID][PID][Size][L_RPM_L][L_RPM_H][R_RPM_L][R_RPM_H][Flags...][Checksum]
 */
int MD4WDHardware::sendVelocityCommand(MotorDriver& driver)
{
    if (!serial_.isOpen())
    {
        return FAIL_4WD;
    }

    uint8_t packet[MAX_PACKET_SIZE_4WD];
    uint8_t checksum = 0;

    // 패킷 헤더
    packet[0] = id_mdt_;              // RMID: 수신자 (183 = MDT)
    packet[1] = id_mdui_;             // TMID: 송신자 (184 = PC)
    packet[2] = driver.id;            // 모터 드라이버 ID
    packet[3] = PID_PNT_VEL_CMD_4WD;  // PID: 207
    packet[4] = 9;                    // DataSize

    IByte4WD left_bytes = short2Byte(driver.cmd_left_rpm);
    IByte4WD right_bytes = short2Byte(driver.cmd_right_rpm);

    packet[5] = left_bytes.low;       // left RPM (L)
    packet[6] = left_bytes.high;      // left RPM (H)
    packet[7] = right_bytes.low;      // right RPM (L)
    packet[8] = right_bytes.high;     // right RPM (H)
    packet[9] = REQUEST_PNT_MAIN_DATA_4WD;  // 데이터 요청 플래그
    packet[10] = 0;                   // left direction
    packet[11] = 0;                   // right direction
    packet[12] = 0;                   // setting
    packet[13] = 0;                   // timeout

    // 체크섬 계산
    for (int i = 0; i <= 13; ++i)
    {
        checksum += packet[i];
    }
    packet[14] = ~checksum + 1;       // Checksum (2의 보수)

    try
    {
        serial_.write(packet, 15);
        serial_.flush();
    }
    catch (const serial::IOException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                     "[WRITE] Driver %d write error: %s", driver.id, e.what());
        driver.error_count++;
        return FAIL_4WD;
    }

    return SUCCESS_4WD;
}

int MD4WDHardware::receiveData()
{
    if (!serial_.isOpen())
    {
        return FAIL_4WD;
    }

    size_t available = serial_.available();
    if (available == 0)
    {
        return SUCCESS_4WD;
    }

    uint8_t buffer[MAX_DATA_SIZE_4WD];
    size_t read_size = std::min(available, static_cast<size_t>(MAX_DATA_SIZE_4WD));
    
    try
    {
        serial_.read(buffer, read_size);
    }
    catch (const serial::IOException & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "[READ] Error: %s", e.what());
        return FAIL_4WD;
    }

    analyzeReceivedData(buffer, read_size);
    return SUCCESS_4WD;
}

int MD4WDHardware::analyzeReceivedData(uint8_t* buffer, size_t size)
{
    for (size_t j = 0; j < size; ++j)
    {
        if (comm_.packet_num >= MAX_PACKET_SIZE_4WD)
        {
            comm_.step = 0;
            comm_.packet_num = 0;
            comm_.checksum = 0;
            continue;
        }

        switch (comm_.step)
        {
            case 0:  // RMID/TMID 확인
                if (buffer[j] == id_mdui_ || buffer[j] == id_mdt_)
                {
                    comm_.checksum += buffer[j];
                    comm_.recv_buf[comm_.packet_num++] = buffer[j];
                    if (comm_.packet_num >= 2) comm_.step++;
                }
                else
                {
                    comm_.step = 0;
                    comm_.packet_num = 0;
                    comm_.checksum = 0;
                }
                break;

            case 1:  // 모터 드라이버 ID 확인
                if (buffer[j] == front_driver_.id || buffer[j] == rear_driver_.id)
                {
                    comm_.current_driver_id = buffer[j];
                    comm_.checksum += buffer[j];
                    comm_.recv_buf[comm_.packet_num++] = buffer[j];
                    comm_.step++;
                }
                else
                {
                    comm_.step = 0;
                    comm_.packet_num = 0;
                    comm_.checksum = 0;
                }
                break;

            case 2:  // PID 저장
                comm_.checksum += buffer[j];
                comm_.recv_buf[comm_.packet_num++] = buffer[j];
                comm_.step++;
                break;

            case 3:  // DataSize 저장
                comm_.max_data_num = buffer[j];
                comm_.data_num = 0;
                comm_.checksum += buffer[j];
                comm_.recv_buf[comm_.packet_num++] = buffer[j];
                comm_.step++;
                break;

            case 4:  // 데이터 수신
                comm_.recv_buf[comm_.packet_num++] = buffer[j];
                comm_.checksum += buffer[j];
                comm_.data_num++;

                if (comm_.data_num >= MAX_DATA_SIZE_4WD)
                {
                    comm_.step = 0;
                    comm_.packet_num = 0;
                    comm_.checksum = 0;
                    break;
                }

                if (comm_.data_num >= comm_.max_data_num)
                {
                    comm_.step++;
                }
                break;

            case 5:  // 체크섬 확인
                comm_.checksum += buffer[j];
                comm_.recv_buf[comm_.packet_num++] = buffer[j];

                if (comm_.checksum == 0)
                {
                    comm_.packet_ok = true;
                }

                comm_.step = 0;
                comm_.packet_num = 0;
                comm_.checksum = 0;
                break;

            default:
                comm_.step = 0;
                comm_.packet_num = 0;
                comm_.checksum = 0;
                break;
        }

        if (comm_.packet_ok)
        {
            comm_.packet_ok = false;
            processReceivedData(comm_.current_driver_id);
        }
    }

    return SUCCESS_4WD;
}

void MD4WDHardware::processReceivedData(uint8_t driver_id)
{
    uint8_t pid = comm_.recv_buf[3];

    if (pid == PID_MAIN_DATA_4WD)
    {
        uint8_t* data = &comm_.recv_buf[5];
        
        MotorDriver* driver = nullptr;
        if (driver_id == front_driver_.id)
        {
            driver = &front_driver_;
        }
        else if (driver_id == rear_driver_.id)
        {
            driver = &rear_driver_;
        }
        
        if (driver)
        {
            driver->fb_left_rpm = byte2Short(data[0], data[1]);
            driver->fb_left_position = byte2Long(data[2], data[3], data[4], data[5]);
            driver->fb_right_rpm = byte2Short(data[8], data[9]);
            driver->fb_right_position = byte2Long(data[10], data[11], data[12], data[13]);
        }
    }
}

/* ============================================================================
 * 단위 변환 함수들
 * ============================================================================ */

int16_t MD4WDHardware::radPerSecToRpm(double rad_per_sec)
{
    return static_cast<int16_t>(rad_per_sec * 60.0 / (2.0 * M_PI));
}

double MD4WDHardware::rpmToRadPerSec(int16_t rpm)
{
    return rpm * (2.0 * M_PI) / 60.0;
}

double MD4WDHardware::tickToRad(int32_t tick)
{
    return tick * tick_to_rad_;
}

/* ============================================================================
 * 바이트 변환 유틸리티
 * ============================================================================ */

IByte4WD MD4WDHardware::short2Byte(int16_t value)
{
    IByte4WD result;
    result.low = value & 0xFF;
    result.high = (value >> 8) & 0xFF;
    return result;
}

int16_t MD4WDHardware::byte2Short(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>(low | (high << 8));
}

int32_t MD4WDHardware::byte2Long(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return static_cast<int32_t>(b1 | (b2 << 8) | (b3 << 16) | (b4 << 24));
}

/* ============================================================================
 * 디버그/에러 로깅 함수들
 * ============================================================================ */

void MD4WDHardware::printSerialStatus()
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║              Serial Port Status (4WD)                        ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╠══════════════════════════════════════════════════════════════╣");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Port:       %-47s ║", port_.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Baudrate:   %-47d ║", baudrate_);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Is Open:    %-47s ║", serial_.isOpen() ? "YES" : "NO");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Front Driver ID: %-42d ║", front_driver_.id);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ Rear Driver ID:  %-42d ║", rear_driver_.id);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");
}

void MD4WDHardware::printDriverStatus(const MotorDriver& driver, const std::string& name)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "[%s] ID=%d | L_RPM=%d R_RPM=%d | L_Pos=%d R_Pos=%d",
        name.c_str(), driver.id,
        driver.fb_left_rpm, driver.fb_right_rpm,
        driver.fb_left_position, driver.fb_right_position);
}

void MD4WDHardware::printAllWheelStatus()
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║              4WD Wheel Status                                ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╠══════════════════════════════════════════════════════════════╣");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ FL: pos=%.3f rad, vel=%.3f rad/s                            ║", 
        hw_positions_[FRONT_LEFT], hw_velocities_[FRONT_LEFT]);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ FR: pos=%.3f rad, vel=%.3f rad/s                            ║", 
        hw_positions_[FRONT_RIGHT], hw_velocities_[FRONT_RIGHT]);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ RL: pos=%.3f rad, vel=%.3f rad/s                            ║", 
        hw_positions_[REAR_LEFT], hw_velocities_[REAR_LEFT]);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "║ RR: pos=%.3f rad, vel=%.3f rad/s                            ║", 
        hw_positions_[REAR_RIGHT], hw_velocities_[REAR_RIGHT]);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");
}

void MD4WDHardware::printErrorReport(const std::string& function_name, 
                                      const std::string& error_msg,
                                      int error_code)
{
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║                    ⚠️  ERROR REPORT (4WD)  ⚠️                  ║");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "╠══════════════════════════════════════════════════════════════╣");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║ Function:   %-47s ║", function_name.c_str());
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║ Error:      %-47s ║", error_msg.c_str());
    if (error_code != 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
            "║ Error Code: %-47d ║", error_code);
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
            "║ System:     %-47s ║", std::strerror(error_code));
    }
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "╠══════════════════════════════════════════════════════════════╣");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║ Possible Solutions:                                          ║");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║  1. Check if device is connected: ls -la /dev/ttyMotor       ║");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║  2. Check permissions: sudo chmod 666 /dev/ttyMotor          ║");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║  3. Verify both motor drivers are powered and connected      ║");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "║  4. Check RS-485 bus termination and wiring                  ║");
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
        "╚══════════════════════════════════════════════════════════════╝");
}

}  // namespace md_hardware

/* ============================================================================
 * 플러그인 등록
 * ============================================================================ */
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    md_hardware::MD4WDHardware,
    hardware_interface::SystemInterface
)
