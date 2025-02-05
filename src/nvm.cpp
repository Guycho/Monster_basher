#include "nvm.h"

NVM::NVM() {
    // Constructor implementation
}

NVM::~NVM() {
    // Destructor implementation
}

void NVM::init() {
    EEPROM.begin(sizeof(NVMData));
    read();
    if (m_data.steering_mixer_data.motor_speed[R] != m_data.steering_mixer_data.motor_speed[R] ||
        m_data.steering_mixer_data.motor_speed[L] != m_data.steering_mixer_data.motor_speed[L]) {
        m_data.steering_mixer_data.motor_speed[R] = 0;
        m_data.steering_mixer_data.motor_speed[L] = 0;
        write();
    }
}

void NVM::write() {
    EEPROM.put(0, m_data);
    EEPROM.commit();
}

void NVM::read() { EEPROM.get(0, m_data); }

void NVM::check_for_changes() {
    if (memcmp(&m_data, &m_last_data, sizeof(NVMData)) != 0) {
        write();
        m_last_data = m_data;  // Update the last known state
    }
}

NVMData NVM::get_data() const { return m_data; }

void NVM::set_data(const NVMData& data) {
    m_data = data;
    check_for_changes();
}