#include "PID.h"

PID::PID() {
    // Constructor implementation
}

PID::~PID() {
    // Destructor implementation
}
void PID::init(const PIDConfig &config) {
    m_timer = new Chrono(Chrono::MICROS);
    m_timer->start();
    m_kp = config.kp;
    m_ki = config.ki;
    m_kd = config.kd;
    m_min_output = -config.max_output;
    m_max_output = config.max_output;
    m_max_integral = ((config.integral_percentage / 1e2) * m_max_output) / m_ki;
    m_min_integral = -m_max_integral;
    m_low_pass_alpha = config.low_pass_alpha;
    m_high_pass_alpha = config.high_Pass_alpha;
    m_use_filters = config.use_filters;
    m_output = 0;
}

float PID::compute(float set_point, float measured_value) {
    m_set_point = set_point;
    m_measured_value = measured_value;
    m_error = m_set_point - m_measured_value;
    // Get the elapsed time in seconds
    m_dt = m_timer->elapsed() / 1e6;
    if (m_dt == 0) return m_output;
    m_timer->restart();  // Restart the timer for the next iteration
    // Update integral term with anti-windup
    if (m_enable_integral == true) {
        m_integral += m_error * m_dt;
        if (m_integral > m_max_integral) m_integral = m_max_integral;
        if (m_integral < m_min_integral) m_integral = m_min_integral;
    }
    // Calculate derivative term
    float derivative = (m_error - m_previous_error) / m_dt;
    m_previous_error = m_error;

    // Apply high-pass filter to the input
    float filtered_input = 0;
    if (m_use_filters == true) {
        filtered_input = m_high_pass_alpha * (measured_value - m_previous_input);
        m_previous_input = measured_value;
    }

    // Compute the PID output using the filtered input
    m_kp_v = m_kp * m_error;
    m_ki_v = m_ki * m_integral;
    m_kd_v = m_kd * derivative;
    float output = m_kp_v + m_ki_v + m_kd_v + filtered_input;

    // Apply low-pass filter to the output
    if (m_use_filters == true) {
        output = m_low_pass_alpha * output + (1 - m_low_pass_alpha) * m_previous_output;
        m_previous_output = output;
    }
    // Clamp the final output
    if (output > m_max_output) output = m_max_output;
    if (output < m_min_output) output = m_min_output;
    m_output = output;
    return m_output;
}

void PID::enable_integral(bool enable) { m_enable_integral = enable; }

void PID::reset_pid() {
    m_integral = 0;
    m_previous_error = 0;
    m_previous_output = 0;
    m_previous_input = 0;
}

void PID::get_values(float &set_point, float &measured_value, float &kp_v, float &ki_v, float &kd_v,
  float &dt, float &error, float &integral) {
    set_point = m_set_point;
    measured_value = m_measured_value;
    kp_v = m_kp_v;
    ki_v = m_ki_v;
    kd_v = m_kd_v;
    dt = m_dt;
    error = m_error;
    integral = m_integral;
}