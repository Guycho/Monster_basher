#include "steering_mixer.h"

SteeringMixer::SteeringMixer() {}
SteeringMixer::~SteeringMixer() {}

void SteeringMixer::init(const SteeringMixerConfig &config) {
    m_mav_bridge = *config.mav_bridge;
    m_deadband = config.deadband;
    for (int i = 0; i < Config::num_steering; i++) {
        m_pin[i] = config.pin[i];
        m_min_pulse[i] = config.min_pulse[i];
        m_max_pulse[i] = config.max_pulse[i];
    }
}

void SteeringMixer::run(SteeringMixerData &Steering_mixer_data) {
    m_steering_mixer_data = Steering_mixer_data;
    for (int i = 0; i < Config::num_steering; i++) {
        MotorSpeed motor_speed;
        motor_speed.motor_pin = m_pin[i];
        float temp_value = Calcs::calc_dead_band(Steering_mixer_data.motor_speed[i],
          Config::max_percentage, m_deadband);
        float temp_output_value = Calcs::map_float(Steering_mixer_data.motor_speed[i],
          Config::min_percentage, Config::max_percentage, m_min_pulse[i], m_max_pulse[i]);
        float final_value =
          Calcs::constrain_float(temp_output_value, m_min_pulse[i], m_max_pulse[i]);
        motor_speed.motor_value = final_value;
        m_mav_bridge.set_motor_speed(motor_speed);
    }
}

uint8_t SteeringMixer::get_num_of_steering_modes() { return NUM_STEERING_MODES; }

SteeringMixerData SteeringMixer::get_steering_data() { return m_steering_mixer_data; }
