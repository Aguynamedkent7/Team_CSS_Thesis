#pragma once
#include <windows.h>

class VJoyFeeder {
public:
    VJoyFeeder(UINT rID = 1);
    ~VJoyFeeder();
    
    bool init();
    
    // Values range from -1.0 to 1.0 (steering) and 0.0 to 1.0 (pedals)
    void sendInputs(float steering, float gas, float brake);

private:
    UINT m_deviceID;
    bool m_initialized;
};
