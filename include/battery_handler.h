#ifndef BATTERY_HANDLER_H
#define BATTERY_HANDLER_H

#include <Arduino.h>
#include <Chrono.h>

#include "mav_bridge.h"

enum BatteryStatus { BATTERY_NORMAL = 0, BATTERY_LOW = 1, BATTERY_CRITICAL = 2, USB_POWER = 3 };

struct BatteryHandlerConfig {
    MavBridge *mav_bridge;
    uint32_t check_interval;
    float low_voltage_threshold;
    uint16_t low_voltage_timeout;
    float critical_voltage_threshold;
    uint16_t critical_voltage_timeout;
    float usb_power_threshold;
    uint16_t usb_power_timeout;
};

class BatteryHandler {
   public:
    BatteryHandler();
    ~BatteryHandler();

    void init(const BatteryHandlerConfig &config);
    void run();
    BatteryStatus get_battery_status();
    float get_voltage();

   private:
    uint8_t auto_detect_num_cells();
    void check_voltage();
    void update_battery_status();

    Chrono m_check_timer;
    Chrono m_low_voltage_timer;
    Chrono m_critical_voltage_timer;
    Chrono m_usb_power_timer;
    MavBridge *m_mav_bridge;

    uint32_t m_check_interval;
    float m_low_voltage_threshold;
    uint16_t m_low_voltage_timeout;
    float m_critical_voltage_threshold;
    uint16_t m_critical_voltage_timeout;
    float m_usb_power_threshold;
    uint16_t m_usb_power_timeout;
    float m_voltage;
    BatteryStatus m_battery_status;
};

#endif  // BATTERY_HANDLER_H