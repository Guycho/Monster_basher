#include "mav_bridge.h"

MavBridge::MavBridge() {}
MavBridge::~MavBridge() {}
void MavBridge::init(const MavBridgeConfig &config) {
    m_serial = config.serial;
    m_baudrate = config.baudrate;
    m_system_id = config.system_id;
    m_component_id = config.component_id;
    m_message_rate_timeout_level_1 = 1e3 / config.message_rate_level_1;
    m_message_rate_timeout_level_2 = 1e3 / config.message_rate_level_2;
    m_arm_request_timeout = 1e3 / config.arm_request_timeout;

    m_is_alive_timeout = config.is_alive_timeout;

    m_serial->begin(m_baudrate);
    m_is_alive_timer.start();
    m_message_rate_timer_level_1.start();
    m_message_rate_timer_level_2.start();
    m_arm_request_timer.start();
    set_group_messages();
    set_messages_rate();
}

void MavBridge::run() {
    handle_arm_state();
    // handle_message_requests();
    m_mavlink_data.is_alive = m_is_alive_timer.hasPassed(m_is_alive_timeout);
    mavlink_message_t msg;
    mavlink_status_t status;

    uint16_t len = m_serial->available();
    uint8_t buf[len];
    m_serial->readBytes(buf, len);
    for (uint16_t i = 0; i < len; i++) {
        uint8_t c = buf[i];
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                m_is_alive_timer.restart();
                m_is_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
            } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&msg, &att);
                m_mavlink_data.inertial_data.orientation.x = Calcs::rad_to_deg(att.roll);
                m_mavlink_data.inertial_data.orientation.y = Calcs::rad_to_deg(att.pitch);
                m_mavlink_data.inertial_data.orientation.z = Calcs::rad_to_deg(att.yaw);
                m_mavlink_data.inertial_data.gyro.x = Calcs::rad_to_deg(att.rollspeed);
                m_mavlink_data.inertial_data.gyro.y = Calcs::rad_to_deg(att.pitchspeed);
                m_mavlink_data.inertial_data.gyro.z = Calcs::rad_to_deg(att.yawspeed);
            } else if (msg.msgid == MAVLINK_MSG_ID_SCALED_IMU) {
                mavlink_scaled_imu_t imu;
                mavlink_msg_scaled_imu_decode(&msg, &imu);
                m_mavlink_data.inertial_data.acceleration.x = imu.xacc / 1e2;
                m_mavlink_data.inertial_data.acceleration.y = imu.yacc / 1e2;
                m_mavlink_data.inertial_data.acceleration.z = imu.zacc / 1e2;
            } else if (msg.msgid == MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4) {
                mavlink_esc_telemetry_1_to_4_t esc;
                mavlink_msg_esc_telemetry_1_to_4_decode(&msg, &esc);
                for (uint8_t i = 0; i < Config::num_wheels; i++) {
                    m_mavlink_data.four_motor_speed.motor_rpm[i] = esc.rpm[i];
                }
            } else if (msg.msgid == MAVLINK_MSG_ID_VIBRATION) {
                mavlink_vibration_t vib;
                mavlink_msg_vibration_decode(&msg, &vib);
                m_mavlink_data.vibration.x = vib.vibration_x;
                m_mavlink_data.vibration.y = vib.vibration_y;
                m_mavlink_data.vibration.z = vib.vibration_z;
            } else if (msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
                mavlink_battery_status_t battery;
                mavlink_msg_battery_status_decode(&msg, &battery);
                m_mavlink_data.battery_voltage = battery.voltages[0] / 1e3;
            }
        }
    }
}
void MavBridge::set_message_rate(uint32_t msg_id, uint8_t rate) {
    MavMsg m_mav_msg;
    m_mav_msg.system_id = m_system_id;
    m_mav_msg.component_id = m_component_id;
    m_mav_msg.msg_id = MAV_CMD_SET_MESSAGE_INTERVAL;
    m_mav_msg.params[0] = msg_id;
    m_mav_msg.params[1] = 1e6 / rate;
    send_mavlink_message(m_mav_msg);
}

void MavBridge::set_messages_rate() {
    for (uint8_t i = 0; i < Config::MavlinkBridge::num_messages_per_group; i++) {
        if (m_message_group_level_1[i] != UINT32_MAX) {
            set_message_rate(m_message_group_level_1[i],
              Config::MavlinkBridge::message_rate_level_1);
        }
    }
    for (uint8_t i = 0; i < Config::MavlinkBridge::num_messages_per_group; i++) {
        if (m_message_group_level_2[i] != UINT32_MAX) {
            set_message_rate(m_message_group_level_2[i],
              Config::MavlinkBridge::message_rate_level_2);
        }
    }
}

void MavBridge::handle_message_requests() {
    if (m_message_rate_timer_level_1.hasPassed(m_message_rate_timeout_level_1, true)) {
        for (uint8_t i = 0; i < Config::MavlinkBridge::num_messages_per_group; i++) {
            if (m_message_group_level_1[i] != UINT32_MAX) {
                request_message(m_message_group_level_1[i]);
            }
        }
    }
    if (m_message_rate_timer_level_2.hasPassed(m_message_rate_timeout_level_2, true)) {
        for (uint8_t i = 0; i < Config::MavlinkBridge::num_messages_per_group; i++) {
            if (m_message_group_level_2[i] != UINT32_MAX) {
                request_message(m_message_group_level_2[i]);
            }
        }
    }
}

void MavBridge::set_group_messages() {
    for (uint8_t i = 0; i < Config::MavlinkBridge::num_messages_per_group; i++) {
        m_message_group_level_1[i] = UINT32_MAX;
        m_message_group_level_2[i] = UINT32_MAX;
    }
    m_message_group_level_1[0] = MAVLINK_MSG_ID_ATTITUDE;
    m_message_group_level_1[1] = MAVLINK_MSG_ID_SCALED_IMU;
    m_message_group_level_1[2] = MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4;
    m_message_group_level_1[3] = MAVLINK_MSG_ID_VIBRATION;

    m_message_group_level_2[0] = MAVLINK_MSG_ID_HEARTBEAT;
    m_message_group_level_2[1] = MAVLINK_MSG_ID_BATTERY_STATUS;
}

void MavBridge::set_motor_speed(MotorSpeed motor_speed) {
    MavMsg m_mav_msg;
    m_mav_msg.system_id = m_system_id;
    m_mav_msg.component_id = m_component_id;
    m_mav_msg.msg_id = MAV_CMD_DO_SET_SERVO;
    m_mav_msg.params[0] = motor_speed.motor_pin;
    m_mav_msg.params[1] = motor_speed.motor_value;
    send_mavlink_message(m_mav_msg);
}

void MavBridge::set_arm_state(bool arm_state) { m_arm_requested = arm_state; }

void MavBridge::handle_arm_state() {
    if (m_arm_request_timer.hasPassed(m_arm_request_timeout, true) &&
        (m_is_armed != m_arm_requested)) {
        MavMsg m_mav_msg;
        m_mav_msg.system_id = m_system_id;
        m_mav_msg.component_id = m_component_id;
        m_mav_msg.msg_id = MAV_CMD_COMPONENT_ARM_DISARM;
        m_mav_msg.params[0] = m_arm_requested ? 1 : 0;
        send_mavlink_message(m_mav_msg);
    }
}

void MavBridge::request_message(uint32_t msg_id) {
    MavMsg m_mav_msg;
    m_mav_msg.system_id = m_system_id;
    m_mav_msg.component_id = m_component_id;
    m_mav_msg.msg_id = MAV_CMD_REQUEST_MESSAGE;
    m_mav_msg.params[0] = msg_id;
    send_mavlink_message(m_mav_msg);
}

void MavBridge::send_mavlink_message(const MavMsg &mav_msg) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the COMMAND_LONG message
    mavlink_msg_command_long_pack(mav_msg.system_id,  // Your system ID
      mav_msg.component_id,                           // Your component ID
      &msg,
      mav_msg.system_id,     // Target system ID
      mav_msg.component_id,  // Target component ID
      mav_msg.msg_id,        // Command ID
      0,                     // Confirmation
      mav_msg.params[0],     // Param 1
      mav_msg.params[1],     // Param 2
      mav_msg.params[2],     // Param 3
      mav_msg.params[3],     // Param 4
      mav_msg.params[4],     // Param 5
      mav_msg.params[5],     // Param 6
      mav_msg.params[6]      // Param 7
    );

    // Serialize the message
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message over the serial interface
    m_serial->write(buf, len);
}

MavlinkData MavBridge::get_mavlink_data() { return m_mavlink_data; }