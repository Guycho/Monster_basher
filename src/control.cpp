#include "control.h"

Control::Control() {}
Control::~Control() {}

void Control::init(const ControlConfig &config) {
    m_hb_timer.start();
    m_mav_bridge = config.mav_bridge;
    m_steering_mixer = config.steering_mixer;
    m_wheels_mixer = config.wheels_mixer;
    m_pid = config.pid;
    m_input_controller = config.input_controller;
    m_battery_handler = config.battery_handler;
    m_transceiver = config.transceiver;
    m_arm_led_pin = config.arm_led_pin;
    m_arm_enabled = false;
    m_throttle = 0;
    m_steering = 0;
    NUM_STEERING_MODES = m_steering_mixer->get_num_of_steering_modes();
    NUM_DRIVE_MODES = m_wheels_mixer->get_num_of_drive_modes();
    pinMode(m_arm_led_pin, OUTPUT);
}

void Control::run() {
    update_mavlink_data();
    update_battery_status();
    update_input_data();

    if (m_input_data.new_data) {
        handle_new_input_data();
    }

    if (m_hb_timer.hasPassed(Config::hb_timeout)) {
        handle_heartbeat_timeout();
    }

    if (m_arm_enabled) {
        handle_steering_mode();
        handle_drive_mode();
        handle_locks();
    } else {
        disable_motors();
    }
    digitalWrite(m_arm_led_pin, m_arm_enabled);
    apply_multiplier(m_steering_mixer_data);
    m_steering_mixer->run(m_steering_mixer_data);
    m_wheels_mixer->run(m_wheels_mixer_data);
    handle_telemetry_data();
}

void Control::update_mavlink_data() { m_mavlink_data = m_mav_bridge->get_mavlink_data(); }

void Control::update_input_data() { m_input_data = m_input_controller->get_input_data(); }

void Control::handle_telemetry_data() {
    m_telemetry_data.battery_voltage = m_battery_handler->get_voltage();
    m_telemetry_data.arm_state = m_arm_enabled;
    m_telemetry_data.steering_mode = m_steering_mode;
    m_telemetry_data.drive_mode = m_drive_mode;
    m_telemetry_data.battery_status = m_battery_status;
    for (int i = 0; i < Config::num_wheels; i++) {
        m_telemetry_data.motors_throttle[i] = m_wheels_mixer_data.motor_speed[i];
        m_telemetry_data.motors_rpm[i] = m_mavlink_data.four_motor_speed.motor_rpm[i];
    }
    for (int i = 0; i < Config::num_steering; i++) {
        m_telemetry_data.steering_values[i] = m_steering_mixer_data.motor_speed[i];
    }
    m_telemetry_data.g_force_x = m_mavlink_data.inertial_data.acceleration.x;
    m_telemetry_data.g_force_y = m_mavlink_data.inertial_data.acceleration.y;
    m_telemetry_data.rotation_rate_z = m_mavlink_data.inertial_data.gyro.z;
    m_transceiver->set_telemetry_data(m_telemetry_data);
}

void Control::update_battery_status() {
    m_battery_status = m_battery_handler->get_battery_status();
    handle_battery_status();
}

void Control::handle_battery_status() {
    if (m_battery_status == BATTERY_CRITICAL) {
        m_arm_enabled = false;
        m_mav_bridge->set_arm_state(false);
        m_throttle = 0;
        m_steering = 0;
        m_battery_ok = false;
    } else if (m_battery_status == BATTERY_LOW) {
        m_battery_ok = false;
    } else {
        m_battery_ok = true;
    }
}

void Control::handle_new_input_data() {
    m_was_safe = !m_input_data.arm_switch ? true : m_was_safe;
    m_ready_to_arm =
      m_was_safe && m_input_data.throttle == 0 && m_input_data.steering == 0 && m_battery_ok;
    if (m_input_data.arm_switch != m_arm_enabled) {
        m_arm_enabled = m_input_data.arm_switch && m_ready_to_arm;
        m_mav_bridge->set_arm_state(m_arm_enabled);
    }
    if (m_input_data.steering_mode_toggle) {
        m_steering_mode = (m_steering_mode + 1) % NUM_STEERING_MODES;
        m_pid->reset_pid();
    }
    if (m_input_data.drive_mode_toggle) {
        m_drive_mode = (m_drive_mode + 1) % NUM_DRIVE_MODES;
    }
    m_steering = m_input_data.steering;
    m_throttle = m_input_data.throttle;
    m_lock_rear_right = m_input_data.lock_rear_right || m_steering == Config::max_percentage;
    m_lock_rear_left = m_input_data.lock_rear_left || m_steering == Config::min_percentage;
    m_hb_timer.restart();
}

void Control::handle_heartbeat_timeout() {
    m_arm_enabled = false;
    m_mav_bridge->set_arm_state(false);
    m_throttle = 0;
    m_steering = 0;
}

void Control::handle_steering_mode() {
    float desired_omega = 0.0f;
    float pid_output = 0.0f;
    float current_omega = 0.0f;
    switch (m_steering_mode) {
        case NORMAL:
            m_steering_mixer_data.motor_speed[R] = m_steering;
            m_steering_mixer_data.motor_speed[L] = m_steering;
            break;
        case GYRO:
            desired_omega = m_steering;
            current_omega = Calcs::constrain_float(
              Calcs::map_float(m_mavlink_data.inertial_data.gyro.z, -Config::max_omega,
                Config::max_omega, Config::min_percentage, Config::max_percentage),
              Config::min_percentage, Config::max_percentage);
            pid_output = m_pid->compute(desired_omega, current_omega);
            m_steering_mixer_data.motor_speed[R] = pid_output;
            m_steering_mixer_data.motor_speed[L] = pid_output;
            break;
        default:
            m_steering_mixer_data.motor_speed[R] = 0;
            m_steering_mixer_data.motor_speed[L] = 0;
            break;
    }
}

void Control::handle_drive_mode() {
    uint8_t current_drive_mode = m_drive_mode;
    if (m_throttle < 0) {
        current_drive_mode = AWD;
    }
    switch (current_drive_mode) {
        case AWD:
            m_wheels_mixer_data.motor_speed[FR] = m_throttle;
            m_wheels_mixer_data.motor_speed[RR] = m_throttle;
            m_wheels_mixer_data.motor_speed[RL] = m_throttle;
            m_wheels_mixer_data.motor_speed[FL] = m_throttle;
            break;
        case CS:
            m_wheels_mixer_data.motor_speed[FR] = m_throttle * Config::cs_ratio;
            m_wheels_mixer_data.motor_speed[RR] = m_throttle;
            m_wheels_mixer_data.motor_speed[RL] = m_throttle;
            m_wheels_mixer_data.motor_speed[FL] = m_throttle * Config::cs_ratio;
            break;
        case RWD:
            m_wheels_mixer_data.motor_speed[FR] = 0;
            m_wheels_mixer_data.motor_speed[RR] = m_throttle;
            m_wheels_mixer_data.motor_speed[RL] = m_throttle;
            m_wheels_mixer_data.motor_speed[FL] = 0;
            break;
        default:
            m_wheels_mixer_data.motor_speed[FR] = 0;
            m_wheels_mixer_data.motor_speed[RR] = 0;
            m_wheels_mixer_data.motor_speed[RL] = 0;
            m_wheels_mixer_data.motor_speed[FL] = 0;
            break;
    }
}

void Control::handle_locks() {
    if (m_lock_rear_right) {
        m_wheels_mixer_data.motor_speed[RR] = 0;
    }
    if (m_lock_rear_left) {
        m_wheels_mixer_data.motor_speed[RL] = 0;
    }
}

void Control::disable_motors() {
    m_steering_mixer_data.motor_speed[R] = 0;
    m_steering_mixer_data.motor_speed[L] = 0;
    m_wheels_mixer_data.motor_speed[FR] = 0;
    m_wheels_mixer_data.motor_speed[RR] = 0;
    m_wheels_mixer_data.motor_speed[RL] = 0;
    m_wheels_mixer_data.motor_speed[FL] = 0;
}

void Control::apply_multiplier(SteeringMixerData &steering_mixer_data) {
    float adjustment_value =
      (steering_mixer_data.motor_speed[R] + steering_mixer_data.motor_speed[L]) *
      Config::steering_r_l_ratio;
    if (steering_mixer_data.motor_speed[R] + steering_mixer_data.motor_speed[L] > 0) {
        steering_mixer_data.motor_speed[L] -= adjustment_value;
    } else {
        steering_mixer_data.motor_speed[R] -= adjustment_value;
    }
}