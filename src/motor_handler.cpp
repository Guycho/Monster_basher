#include "motor_handler.h"

MotorHandler::MotorHandler() {}
MotorHandler::~MotorHandler() {}

void MotorHandler::init(const MotorHandlerConfig &config) {
    m_odrive = config.odrive;
    m_max_torque = m_odrive->getParameterAsFloat("ODrive.motor.config.current_lim");
    m_max_velocity = m_odrive->getParameterAsFloat("ODrive.motor.config.vel_limit");
    m_motor_control_mode = motorControlMode::TORQUE_CONTROL;
    m_cmd = 0.0;
    m_motor_control = false;
}

void MotorHandler::run() {
    handle_motor_control();
    handle_motor_control_mode();
    if (m_motor_control_mode == motorControlMode::TORQUE_CONTROL) {
        float torque = Calcs::map_float(m_cmd, -100, 100, -m_max_torque, m_max_torque);
        m_odrive->setTorque(torque);
    } else if (m_motor_control_mode == motorControlMode::VELOCITY_CONTROL) {
        float velocity = Calcs::map_float(m_cmd, -100, 100, -m_max_velocity, m_max_velocity);
        m_odrive->setVelocity(velocity);
    }
}

void MotorHandler::set_cmd(float cmd) { m_cmd = Calcs::constrain_float(cmd, -100, 100); }

void MotorHandler::set_motor_control(bool on_off) { m_motor_control = on_off; }

void MotorHandler::set_motor_control_mode(motorControlMode mode) { m_motor_control_mode = mode; }

void MotorHandler::handle_motor_control() {
    if (m_motor_control == (m_odrive->getState() == AXIS_STATE_CLOSED_LOOP_CONTROL)) {
        return;
    }
    if (m_motor_control) {
        m_odrive->setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    } else {
        m_odrive->setState(AXIS_STATE_IDLE);
    }
}

void MotorHandler::handle_motor_control_mode() {
    if (m_motor_control_mode == get_control_mode_from_string()) {
        return;
    }
    m_odrive->setParameter("ODrive.Controller.ControlMode",
      control_mode_to_string(m_motor_control_mode));
}

String MotorHandler::control_mode_to_string(motorControlMode mode) {
    switch (mode) {
        case motorControlMode::VOLTAGE_CONTROL:
            return "CONTROL_MODE_VOLTAGE_CONTROL";
        case motorControlMode::TORQUE_CONTROL:
            return "CONTROL_MODE_TORQUE_CONTROL";
        case motorControlMode::VELOCITY_CONTROL:
            return "CONTROL_MODE_VELOCITY_CONTROL";
        case motorControlMode::POSITION_CONTROL:
            return "CONTROL_MODE_POSITION_CONTROL";
        default:
            return "Unknown";
    }
}

motorControlMode MotorHandler::get_control_mode_from_string() {
    String mode_string = m_odrive->getParameterAsString("ODrive.Controller.ControlMode");
    if (mode_string == "CONTROL_MODE_VOLTAGE_CONTROL") {
        return motorControlMode::VOLTAGE_CONTROL;
    } else if (mode_string == "CONTROL_MODE_TORQUE_CONTROL") {
        return motorControlMode::TORQUE_CONTROL;
    } else if (mode_string == "CONTROL_MODE_VELOCITY_CONTROL") {
        return motorControlMode::VELOCITY_CONTROL;
    } else if (mode_string == "CONTROL_MODE_POSITION_CONTROL") {
        return motorControlMode::POSITION_CONTROL;
    } else {
        return motorControlMode::TORQUE_CONTROL;
    }
}
