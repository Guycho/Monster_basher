#ifndef NVM_H
#define NVM_H

#include <EEPROM.h>

#include "steering_mixer.h"

struct NVMData {
    SteeringMixerData steering_mixer_data;
};

class NVM {
   public:
    NVM();                               // Constructor
    ~NVM();                              // Destructor
    void init();                         // Method to start the NVM
    void write();                        // Method to write the NVM
    void read();                         // Method to read the NVM
    void check_for_changes();            // Method to check for changes and write if necessary
    NVMData get_data() const;            // Method to get the NVM data
    void set_data(const NVMData& data);  // Method to set the NVM data

   private:
    NVMData m_data;       // Variable to store the NVM data
    NVMData m_last_data;  // Variable to store the last known state of the EEPROM data
};

#endif  // NVM_H