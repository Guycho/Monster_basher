#include "input_controller.h"

InputController::InputController() {}

// Destructor
InputController::~InputController() {}

void InputController::init(const InputControllerConfig &config) {
    m_transceiver = config.transceiver;
}
InputControllerData InputController::get_input_data() {
    RemoteControllerData remote_data = m_transceiver->get_remote_data();
    InputControllerData input_controller_data;
    input_controller_data.throttle = remote_data.throttle;
    input_controller_data.steering = remote_data.steering;
    input_controller_data.arm_switch = remote_data.edge_switch;
    input_controller_data.steering_mode_toggle = remote_data.ch;
    input_controller_data.drive_mode_toggle = remote_data.sel;
    input_controller_data.lock_rear_right = remote_data.right_arrow;
    input_controller_data.lock_rear_left = remote_data.left_arrow;
    // input_controller_data.write_to_nvm =
    // input_controller_data.trim_r =
    // input_controller_data.trim_l =
    // input_controller_data.trim_throttle =
    // input_controller_data.trim_steering =
    // input_controller_data.trim_direction_r =
    // input_controller_data.trim_direction_l =
    // input_controller_data.trim_direction_f =
    // input_controller_data.trim_direction_b =
    // input_controller_data.reset_trim =
    input_controller_data.new_data = remote_data.new_data;
    return input_controller_data;
}