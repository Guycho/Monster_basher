#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#include "credentials.h"
enum WheelsPositions { RR = 0, FR = 1, RL = 2, FL = 3 };
enum SteeringPositions { L = 0, R = 1 };

namespace Config {
const uint8_t num_wheels = 4;
const uint8_t num_steering = 2;
const float min_percentage = -100.0;
const float max_percentage = 100.0;
const float max_omega = 360;  // deg/s
const float steering_r_l_ratio = 0.1;
const float trim_increment = 0.01;
const float cs_ratio = 0.7;
const uint16_t hb_timeout = 2500;
const uint8_t arm_led_pin = 2;

namespace Transceiver {
extern const uint16_t update_delay_ms;
}

namespace Motor {
extern const float deadband;
namespace Wheel {
extern const uint8_t pin[];
extern const uint16_t min_pulse[];
extern const uint16_t max_pulse[];
}  // namespace Wheel
namespace Steering {
extern const uint8_t pin[];
extern const uint16_t min_pulse[];
extern const uint16_t max_pulse[];
}  // namespace Steering
}  // namespace Motor

namespace PIDController {
extern const float kp;
extern const float ki;
extern const float kd;
extern const float max_output;
extern const float integral_percentage;
extern const float low_pass_alpha;
extern const float high_Pass_alpha;
extern const bool use_filters;
}  // namespace PIDController

namespace MavlinkBridge {
const uint8_t num_messages_per_group = 10;
extern HardwareSerial *serial;
extern const uint32_t baudrate;
extern const uint8_t system_id;
extern const uint8_t component_id;
extern const uint8_t message_rate_level_1;
extern const uint8_t message_rate_level_2;
extern const uint8_t arm_request_rate;
extern const uint16_t is_alive_timeout;
}  // namespace MavlinkBridge

namespace BatteryHandler {
extern const uint32_t check_interval;
extern const float low_voltage_threshold;
extern const uint16_t low_voltage_timeout;
extern const float critical_voltage_threshold;
extern const uint16_t critical_voltage_timeout;
extern const float usb_power_threshold;
extern const uint16_t usb_power_timeout;
}  // namespace BatteryHandler

namespace ESPNow {
extern const uint8_t peer_mac_address[];
extern const bool use_lr;
extern const bool print_debug;
}  // namespace ESPNow
namespace OTAHandler {
const uint8_t num_networks = 3;
const uint16_t timeout_sec = 60;
const bool print_debug = false;
extern const char *credentials[][2];
extern const char *hostname;
}  // namespace OTAHandler

}  // namespace Config

#endif  // CONFIG_H