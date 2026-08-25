#include "ActuatorMapper.h"
#include <algorithm>
#include <cmath>

ActuatorMapper::ActuatorMapper() {}

ActuatorMapper::RawInputs ActuatorMapper::mapToPedals(float target_long_accel, float target_steer_angle, float current_speed_ms) {
    RawInputs inputs;
    
    // Simple P-controller / Direct Mapping
    // This is a naive implementation that ignores gearing, RPM, and aero drag.
    // It assumes max acceleration is ~1g and max braking is ~1.5g.
    float max_accel = 9.81f; 
    float max_decel = -14.7f; 
    
    if (target_long_accel > 0) {
        inputs.gas_pedal = std::clamp(target_long_accel / max_accel, 0.0f, 1.0f);
        inputs.brake_pedal = 0.0f;
    } else {
        inputs.gas_pedal = 0.0f;
        inputs.brake_pedal = std::clamp(target_long_accel / max_decel, 0.0f, 1.0f);
    }
    
    // Assuming max steering angle at the wheels is ~0.5 radians (approx 28 degrees)
    float max_steer_rad = 0.5f; 
    inputs.steering_axis = std::clamp(target_steer_angle / max_steer_rad, -1.0f, 1.0f);
    
    return inputs;
}
