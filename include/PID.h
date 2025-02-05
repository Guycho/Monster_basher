#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <Chrono.h>

struct PIDConfig {
    float kp;
    float ki;
    float kd;
    float max_output;
    float integral_percentage;
    float low_pass_alpha;
    float high_Pass_alpha;
    bool use_filters;
};
class PID {
   public:
    PID();   // Constructor
    ~PID();  // Destructor

    void init(const PIDConfig &config);
    float compute(float set_point, float measured_value);  // Method to compute the PID output
    void reset_pid();                                      // Method to reset the PID controller
    void enable_integral(bool enable);  // Method to enable/disable the integral term
    void get_values(float &set_point, float &measured_value, float &kp_v, float &ki_v, float &kd_v,
      float &dt, float &error, float &integral);

   private:
    Chrono *m_timer;

    float m_kp;  // Proportional coefficient
    float m_ki;  // Integral coefficient
    float m_kd;  // Derivative coefficient

    float m_set_point;       // Set point value
    float m_measured_value;  // Measured value
    float m_output;          // PID output

    float m_dt;  // Time step

    float m_error = 0;           // Error for proportional calculation
    float m_previous_error = 0;  // Previous error for derivative calculation
    float m_integral = 0;        // Integral of the error

    bool m_enable_integral;  // Enable/disable the integral term

    float m_min_integral;  // Minimum integral for anti-windup
    float m_max_integral;  // Maximum integral for anti-windup

    float m_min_output;  // Minimum output for anti-windup
    float m_max_output;  // Maximum output for anti-windup

    float m_low_pass_alpha;   // Low-pass filter coefficient
    float m_high_pass_alpha;  // High-pass filter coefficient

    float m_previous_output;  // Previous output for low-pass filter
    float m_previous_input;   // Previous input for high-pass filter

    bool m_use_filters;

    float m_kp_v;
    float m_ki_v;
    float m_kd_v;
};

#endif  // PID_H