#include <ESP_now_handler.h>
#include <OTAHandler.h>
#include <TelnetStream.h>

#include "PID.h"
#include "battery_handler.h"
#include "config.h"
#include "control.h"
#include "input_controller.h"
#include "mav_bridge.h"
#include "steering_mixer.h"
#include "transceiver.h"
#include "wheels_mixer.h"

ESPNowHandler esp_now_handler(Config::ESPNow::peer_mac_address, Config::ESPNow::use_lr,
  Config::ESPNow::print_debug);
OTAHandler ota_handler(Config::OTAHandler::hostname, Config::OTAHandler::credentials,
  Config::OTAHandler::num_networks, Config::OTAHandler::timeout_sec,
  Config::OTAHandler::print_debug);
Transceiver transceiver;
InputController input_controller;
MavBridge mav_bridge;
SteeringMixer steering_mixer;
WheelsMixer wheels_mixer;
PID pid;
Control control;
BatteryHandler battery_handler;

void setup() {
    Serial.begin(9600);

    esp_now_handler.init();

    ota_handler.init();

    TelnetStream.begin();

    TransceiverConfig transceiver_config = {.update_delay_ms = Config::Transceiver::update_delay_ms,
      .esp_now_handler = &esp_now_handler};
    transceiver.init(transceiver_config);

    InputControllerConfig input_controller_config = {.transceiver = &transceiver};
    input_controller.init(input_controller_config);

    MavBridgeConfig mav_config = {.serial = Config::MavlinkBridge::serial,
      .baudrate = Config::MavlinkBridge::baudrate,
      .system_id = Config::MavlinkBridge::system_id,
      .component_id = Config::MavlinkBridge::component_id,
      .message_rate_level_1 = Config::MavlinkBridge::message_rate_level_1,
      .message_rate_level_2 = Config::MavlinkBridge::message_rate_level_2,
      .arm_request_timeout = Config::MavlinkBridge::arm_request_rate,
      .is_alive_timeout = Config::MavlinkBridge::is_alive_timeout};
    mav_bridge.init(mav_config);

    SteeringMixerConfig steering_config = {.mav_bridge = &mav_bridge,
      .deadband = Config::Motor::deadband};
    for (int i = 0; i < Config::num_steering; i++) {
        steering_config.pin[i] = Config::Motor::Steering::pin[i];
        steering_config.min_pulse[i] = Config::Motor::Steering::min_pulse[i];
        steering_config.max_pulse[i] = Config::Motor::Steering::max_pulse[i];
    }
    steering_mixer.init(steering_config);

    WheelsMixerConfig wheels_config = {.mav_bridge = &mav_bridge,
      .deadband = Config::Motor::deadband};
    for (int i = 0; i < Config::num_wheels; i++) {
        wheels_config.pin[i] = Config::Motor::Wheel::pin[i];
        wheels_config.min_pulse[i] = Config::Motor::Wheel::min_pulse[i];
        wheels_config.max_pulse[i] = Config::Motor::Wheel::max_pulse[i];
    }
    wheels_mixer.init(wheels_config);

    PIDConfig pid_config = {.kp = Config::PIDController::kp,
      .ki = Config::PIDController::ki,
      .kd = Config::PIDController::kd,
      .max_output = Config::PIDController::max_output,
      .integral_percentage = Config::PIDController::integral_percentage,
      .low_pass_alpha = Config::PIDController::low_pass_alpha,
      .high_Pass_alpha = Config::PIDController::high_Pass_alpha,
      .use_filters = Config::PIDController::use_filters};
    pid.init(pid_config);

    BatteryHandlerConfig battery_config = {.mav_bridge = &mav_bridge,
      .check_interval = Config::BatteryHandler::check_interval,
      .low_voltage_threshold = Config::BatteryHandler::low_voltage_threshold,
      .low_voltage_timeout = Config::BatteryHandler::low_voltage_timeout,
      .critical_voltage_threshold = Config::BatteryHandler::critical_voltage_threshold,
      .critical_voltage_timeout = Config::BatteryHandler::critical_voltage_timeout,
      .usb_power_threshold = Config::BatteryHandler::usb_power_threshold,
      .usb_power_timeout = Config::BatteryHandler::usb_power_timeout};

    battery_handler.init(battery_config);

    ControlConfig control_config = {.mav_bridge = &mav_bridge,
      .steering_mixer = &steering_mixer,
      .wheels_mixer = &wheels_mixer,
      .pid = &pid,
      .input_controller = &input_controller,
      .battery_handler = &battery_handler,
      .transceiver = &transceiver,
      .arm_led_pin = Config::arm_led_pin};
    control.init(control_config);
}

void loop() {
    mav_bridge.run();
    transceiver.run();
    battery_handler.run();
    control.run();
    ota_handler.run();
}
