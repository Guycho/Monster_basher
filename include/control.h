#ifndef CONTROL_H
#define CONTROL_H

#include <Chrono.h>

#include "PID.h"
#include "battery_handler.h"
#include "config.h"
#include "input_controller.h"
#include "mav_bridge.h"
#include "steering_mixer.h"
#include "transceiver.h"
#include "utils.h"
#include "wheels_mixer.h"

struct ControlConfig {
    MavBridge *mav_bridge;
    SteeringMixer *steering_mixer;
    WheelsMixer *wheels_mixer;
    PID *pid;
    InputController *input_controller;
    BatteryHandler *battery_handler;
    Transceiver *transceiver;

    uint8_t arm_led_pin;
};

class Control {
   public:
    // Constructor
    Control();

    // Destructor
    ~Control();

    void init(const ControlConfig &config);
    void run();
    void apply_multiplier(SteeringMixerData &steering_mixer_data);

   private:
    void update_mavlink_data();
    void update_input_data();
    void update_battery_status();
    void handle_new_input_data();
    void handle_battery_status();
    void handle_heartbeat_timeout();
    void handle_steering_mode();
    void handle_drive_mode();
    void handle_locks();
    void handle_telemetry_data();
    void disable_motors();

    MavBridge *m_mav_bridge;
    SteeringMixer *m_steering_mixer;
    WheelsMixer *m_wheels_mixer;
    PID *m_pid;
    InputController *m_input_controller;
    BatteryHandler *m_battery_handler;
    Transceiver *m_transceiver;

    Chrono m_hb_timer;
    MavlinkData m_mavlink_data;
    InputControllerData m_input_data;
    SteeringMixerData m_steering_mixer_data;
    WheelsMixerData m_wheels_mixer_data;
    BatteryStatus m_battery_status;
    TelemetryData m_telemetry_data;

    uint8_t m_arm_led_pin;

    uint8_t NUM_STEERING_MODES;
    uint8_t NUM_DRIVE_MODES;

    uint8_t m_steering_mode;
    uint8_t m_drive_mode;

    bool m_ready_to_arm = false;
    bool m_was_safe = false;
    bool m_battery_ok = false;
    bool m_arm_enabled = false;
    float m_throttle = 0;
    float m_steering = 0;
    bool m_lock_rear_right = false;
    bool m_lock_rear_left = false;
};

#endif  // CONTROL_H