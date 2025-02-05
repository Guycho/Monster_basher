#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <ArduinoJson.h>
#include <Chrono.h>
#include <ESP_now_handler.h>
#include <WiFi.h>
#include <data_types.h>
#include <utils.h>

#include <bitset>

#include "config.h"

struct TransceiverConfig {
    uint16_t update_delay_ms;
    ESPNowHandler *esp_now_handler;
};

using namespace DataTypes;

class Transceiver {
   public:
    Transceiver();
    ~Transceiver();

    void init(const TransceiverConfig &config);
    void run();
    RemoteControllerData get_remote_data();
    void set_telemetry_data(const TelemetryData &data);

   private:
    void send_data();
    RemoteControllerData parse_remote_data(const String &data);
    Chrono m_data_timer;
    ESPNowHandler *m_esp_now_handler;

    RemoteControllerData m_remote_controller_data;
    TelemetryData m_telemetry_data;
    String m_remote_data;
    uint16_t m_update_delay_ms;
};

#endif  // TRANSCEIVER_H