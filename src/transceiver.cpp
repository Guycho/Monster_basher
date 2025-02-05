#include "transceiver.h"

Transceiver::Transceiver() {}

Transceiver::~Transceiver() {}

void Transceiver::init(const TransceiverConfig &config) {
    m_esp_now_handler = config.esp_now_handler;
    m_update_delay_ms = config.update_delay_ms;
    m_data_timer.start();
}

void Transceiver::run() {
    if (!m_data_timer.hasPassed(m_update_delay_ms, true)) {
        return;
    }
    m_remote_data = m_esp_now_handler->get_data();
    if (m_remote_data.length() != 0 && Calcs::verify_checksum(m_remote_data)) {
        m_remote_controller_data = parse_remote_data(m_remote_data);
    }
    send_data();
}

RemoteControllerData Transceiver::get_remote_data() {
    RemoteControllerData remote_data = m_remote_controller_data;
    m_remote_controller_data.new_data = false;
    return remote_data;
}

void Transceiver::set_telemetry_data(const TelemetryData &data) { m_telemetry_data = data; }

RemoteControllerData Transceiver::parse_remote_data(const String &data) {
    JsonDocument m_json_data;
    deserializeJson(m_json_data, data);
    RemoteControllerData remote_data;
    const size_t data_size = sizeof(typeof(remote_data)) * 8;
    std::bitset<data_size> bitmask;
    uint8_t num_of_bitmasks = data_size / (sizeof(uint32_t) * 8);
    uint32_t bitmasks[num_of_bitmasks];
    for (uint8_t i = 0; i < num_of_bitmasks; ++i) {
        bitmasks[i] = m_json_data["b"][i];
        for (uint8_t j = 0; j < 32 && (i * 32 + j) < bitmask.size(); ++j) {
            bitmask[i * 32 + j] = (bitmasks[i] & (1UL << j));
        }
    }
    memcpy(&remote_data, &bitmask, sizeof(typeof(remote_data)));
    remote_data.new_data = true;
    return remote_data;
}

void Transceiver::send_data() {
    JsonDocument m_json_data;
    String json;

    const size_t data_size = sizeof(m_telemetry_data) * 8;
    std::bitset<data_size> bitmask;

    uint8_t *data_ptr = reinterpret_cast<uint8_t *>(&m_telemetry_data);

    for (size_t i = 0; i < data_size; ++i) {
        std::bitset<8> byte(data_ptr[i]);
        bitmask |= (std::bitset<data_size>(byte.to_ulong()) << (i * 8));
    }

    uint8_t num_of_bitmasks = data_size / (sizeof(uint32_t) * 8);

    uint32_t bitmasks[num_of_bitmasks];
    for (uint8_t i = 0; i < num_of_bitmasks; ++i) {
        bitmasks[i] = 0;
        for (uint8_t j = 0; j < 32 && (i * 32 + j) < bitmask.size(); ++j) {
            if (bitmask[i * 32 + j]) {
                bitmasks[i] |= (1UL << j);
            }
        }
        m_json_data["b"][i] = bitmasks[i];
    }
    serializeJson(m_json_data, json);

    m_json_data["c"] = Calcs::calc_checksum(json);
    serializeJson(m_json_data, json);
    m_esp_now_handler->send_data(json);
}
