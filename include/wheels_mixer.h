#ifndef WHEELS_MIXER_H
#define WHEELS_MIXER_H

#include "config.h"
#include "mav_bridge.h"

struct WheelsMixerConfig {
    MavBridge *mav_bridge;
    float deadband;
    uint8_t pin[Config::num_wheels];
    uint16_t min_pulse[Config::num_wheels];
    uint16_t max_pulse[Config::num_wheels];
};

struct WheelsMixerData {
    float motor_speed[Config::num_wheels];
};

enum DriveMode { AWD = 0, CS = 1, RWD = 2 };

class WheelsMixer {
   public:
    // Constructor
    WheelsMixer();

    // Destructor
    ~WheelsMixer();

    void init(const WheelsMixerConfig &config);
    void run(WheelsMixerData &wheels_mixer_data);
    WheelsMixerData get_wheels_data();
    uint8_t get_num_of_drive_modes();

   private:
    MavBridge m_mav_bridge;
    WheelsMixerData m_wheels_mixer_data;
    float m_deadband;
    uint8_t m_pin[Config::num_wheels];
    uint16_t m_min_pulse[Config::num_wheels];
    uint16_t m_max_pulse[Config::num_wheels];

    static constexpr DriveMode drive_modes[] = {AWD, CS, RWD};
    static constexpr uint8_t NUM_DRIVE_MODES = sizeof(drive_modes) / sizeof(drive_modes[0]);
};

#endif  // CONTROL_H