#include "config.h"

namespace Config {
namespace Transceiver {
const uint16_t update_delay_ms = 25;
}

namespace Motor {
const float deadband = 1.0;
namespace Wheel {
const uint8_t pin[num_wheels] = {1, 2, 3, 4};
const uint16_t min_pulse[num_wheels] = {1000, 1000, 1000, 1000};
const uint16_t max_pulse[num_wheels] = {2000, 2000, 2000, 2000};
}  // namespace Wheel
namespace Steering {
const uint8_t pin[num_steering] = {7, 8};
const uint16_t min_pulse[num_steering] = {1100, 1100};
const uint16_t max_pulse[num_steering] = {1900, 1900};
}  // namespace Steering
}  // namespace Motor

namespace PIDController {
const float kp = 1;
const float ki = 0.1;
const float kd = 0.01;
const float max_output = 100;
const float integral_percentage = 30;
const float low_pass_alpha = 0.1;
const float high_Pass_alpha = 0.1;
const bool use_filters = true;
}  // namespace PIDController

namespace MavlinkBridge {
HardwareSerial *serial = &Serial2;
const uint32_t baudrate = 921600;
const uint8_t system_id = 1;
const uint8_t component_id = 1;
const uint8_t message_rate_level_1 = 100;
const uint8_t message_rate_level_2 = 4;
const uint8_t arm_request_rate = 4;
const uint16_t is_alive_timeout = 2500;
}  // namespace MavlinkBridge

namespace BatteryHandler {
const uint32_t check_interval = 250;
const float low_voltage_threshold = 3.5;
const uint16_t low_voltage_timeout = 5000;
const float critical_voltage_threshold = 3.3;
const uint16_t critical_voltage_timeout = 2500;
const float usb_power_threshold = 5.5;
const uint16_t usb_power_timeout = 1000;
}  // namespace BatteryHandler

namespace ESPNow {
const uint8_t peer_mac_address[] = {0xA0, 0xDD, 0x6C, 0x03, 0x9E, 0x40};
const bool use_lr = true;
const bool print_debug = false;
}  // namespace ESPNow
namespace OTAHandler {
const char *credentials[num_networks][2] = {{WIFI_SSID1, WIFI_PASSWORD2},
  {WIFI_SSID2, WIFI_PASSWORD2}, {WIFI_SSID3, WIFI_PASSWORD3}};
const char *hostname = "CarOTA";
}  // namespace OTAHandler
}  // namespace Config