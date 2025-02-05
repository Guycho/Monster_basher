#ifndef MAV_BRIDGE_H
#define MAV_BRIDGE_H

#include <Arduino.h>
#include <Chrono.h>
#include <all/mavlink.h>

#include "config.h"
#include "utils.h"
struct MavBridgeConfig {
    HardwareSerial *serial;
    uint32_t baudrate;
    uint8_t system_id;
    uint8_t component_id;
    uint8_t message_rate_level_1;
    uint8_t message_rate_level_2;
    uint8_t arm_request_timeout;
    uint16_t is_alive_timeout;
};

struct Vector3D {
    float x;
    float y;
    float z;
};

struct InertialData {
    Vector3D gyro;
    Vector3D orientation;
    Vector3D acceleration;
};

struct MavMsg {
    uint8_t system_id;
    uint8_t component_id;
    uint16_t msg_id;
    float params[7] = {0, 0, 0, 0, 0, 0, 0};
};

struct MotorSpeed {
    uint8_t motor_pin;
    uint16_t motor_value;
};

struct FourMotorSpeed {
    uint16_t motor_rpm[Config::num_wheels];
};

struct MavlinkData {
    InertialData inertial_data;
    FourMotorSpeed four_motor_speed;
    Vector3D vibration;
    float battery_voltage;
    bool is_alive = false;
};

class MavBridge {
   public:
    MavBridge();
    ~MavBridge();

    void init(const MavBridgeConfig &mav__msg);
    void run();
    void set_motor_speed(MotorSpeed motor_speed);
    void set_arm_state(bool arm_state);
    MavlinkData get_mavlink_data();

   private:
    void set_group_messages();
    void handle_arm_state();
    void handle_message_requests();
    void set_message_rate(uint32_t msg_id, uint8_t rate);
    void set_messages_rate();
    void request_message(uint32_t msg_id);
    void send_mavlink_message(const MavMsg &mav_msg);

    HardwareSerial *m_serial;
    uint32_t m_baudrate;
    uint8_t m_system_id;
    uint8_t m_component_id;
    uint8_t m_message_rate_timeout_level_1;
    uint8_t m_message_rate_timeout_level_2;
    uint8_t m_arm_request_timeout;

    uint16_t m_is_alive_timeout;
    uint32_t m_message_group_level_1[Config::MavlinkBridge::num_messages_per_group];
    uint32_t m_message_group_level_2[Config::MavlinkBridge::num_messages_per_group];

    bool m_is_armed;
    bool m_arm_requested;

    Chrono m_is_alive_timer;
    Chrono m_message_rate_timer_level_1;
    Chrono m_message_rate_timer_level_2;
    Chrono m_arm_request_timer;
    MavlinkData m_mavlink_data;
};

#endif  // MAV_BRIDGE_H