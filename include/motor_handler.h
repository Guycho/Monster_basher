#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include <Arduino.h>
#include <ODriveUART.h>
#include <utils.h>

struct MotorHandlerConfig {
    ODriveUART *odrive;
};

enum motorControlMode {
    VOLTAGE_CONTROL = 0,
    TORQUE_CONTROL = 1,
    VELOCITY_CONTROL = 2,
    POSITION_CONTROL = 3,
};

class MotorHandler {
   public:
    MotorHandler();
    ~MotorHandler();

    void init(const MotorHandlerConfig &config);
    void run();
    void set_cmd(float cmd);
    void set_motor_control_mode(motorControlMode mode);
    void set_motor_control(bool on_off);

   private:
    void handle_motor_control();
    void handle_motor_control_mode();
    String control_mode_to_string(motorControlMode mode);
    motorControlMode MotorHandler::get_control_mode_from_string();
    ODriveUART *m_odrive;

    motorControlMode m_motor_control_mode;
    float m_max_torque;
    float m_max_velocity;

    float m_cmd;
    bool m_motor_control;
};

#endif  // MOTOR_HANDLER_H