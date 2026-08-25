#include "VJoyFeeder.h"
#include <iostream>
// #include <vjoyinterface.h> // Requires vJoy SDK installed and linked

// Note: This is a stub implementation. To fully compile, the vJoy SDK is required.
// You will need to link against vJoyInterface.lib and include vjoyinterface.h.

VJoyFeeder::VJoyFeeder(UINT rID) : m_deviceID(rID), m_initialized(false) {
}

VJoyFeeder::~VJoyFeeder() {
    if (m_initialized) {
        // RelinquishVJD(m_deviceID);
    }
}

bool VJoyFeeder::init() {
    std::cout << "Initializing vJoy device " << m_deviceID << "...\n";
    /*
    if (!vJoyEnabled()) {
        std::cerr << "vJoy driver not enabled: Failed Getting vJoy attributes.\n";
        return false;
    }

    VjdStat status = GetVJDStatus(m_deviceID);
    if (status == VJD_STAT_OWN || (status == VJD_STAT_FREE && AcquireVJD(m_deviceID))) {
        std::cout << "Acquired vJoy device " << m_deviceID << "\n";
        m_initialized = true;
        ResetVJD(m_deviceID);
        return true;
    } else {
        std::cerr << "Failed to acquire vJoy device number " << m_deviceID << "\n";
        return false;
    }
    */
    
    std::cout << "vJoy mocked for now. SDK required.\n";
    m_initialized = true;
    return true;
}

void VJoyFeeder::sendInputs(float steering, float gas, float brake) {
    if (!m_initialized) return;

    // vJoy axes typically range from 1 to 32768
    // Steering: -1.0 to 1.0  -> 1 to 32768
    long xAxis = (long)((steering + 1.0f) / 2.0f * 32767.0f) + 1;
    
    // Gas: 0.0 to 1.0 -> 1 to 32768
    long yAxis = (long)(gas * 32767.0f) + 1;
    
    // Brake: 0.0 to 1.0 -> 1 to 32768
    long zAxis = (long)(brake * 32767.0f) + 1;

    /*
    SetAxis(xAxis, m_deviceID, HID_USAGE_X);
    SetAxis(yAxis, m_deviceID, HID_USAGE_Y);
    SetAxis(zAxis, m_deviceID, HID_USAGE_Z);
    */
    
    // Print for debugging if needed, but not every frame to avoid lag
    // std::cout << "Steer: " << steering << " Gas: " << gas << " Brake: " << brake << "\n";
}
