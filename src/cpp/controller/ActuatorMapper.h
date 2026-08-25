#pragma once

class ActuatorMapper {
public:
    ActuatorMapper();
    
    struct RawInputs {
        float steering_axis; // -1.0 to 1.0
        float gas_pedal;     // 0.0 to 1.0
        float brake_pedal;   // 0.0 to 1.0
    };

    // Maps MPC acceleration target to gas/brake using current speed and target speed
    // simple P-controller approach for now.
    RawInputs mapToPedals(float target_long_accel, float target_steer_angle, float current_speed_ms);
};
