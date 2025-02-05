#ifndef INPUT_CONTROLLER_H
#define INPUT_CONTROLLER_H

#include <Arduino.h>
#include <ArduinoJson.h>

#include "transceiver.h"

struct InputControllerData {
    float throttle;
    float steering;
    bool arm_switch;
    bool steering_mode_toggle;
    bool drive_mode_toggle;
    bool lock_rear_right;
    bool lock_rear_left;
    bool write_to_nvm;
    bool trim_r;
    bool trim_l;
    bool trim_throttle;
    bool trim_steering;
    bool trim_direction_r;
    bool trim_direction_l;
    bool trim_direction_f;
    bool trim_direction_b;
    bool reset_trim;
    bool new_data;
};

struct InputControllerConfig {
    Transceiver *transceiver;
};

class InputController {
   public:
    // Constructor
    InputController();

    // Destructor
    ~InputController();

    void init(const InputControllerConfig &config);
    InputControllerData get_input_data();

   private:
    Transceiver *m_transceiver;
    InputControllerData m_input_data;
};

#endif  // INPUT_CONTROLLER_H