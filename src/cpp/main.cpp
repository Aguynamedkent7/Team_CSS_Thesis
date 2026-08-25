#include <iostream>
#include <thread>
#include <chrono>
#include "connector/ACSharedMemory.h"
#include "connector/VJoyFeeder.h"
#include "controller/SCRController.h"
#include "controller/ActuatorMapper.h"

int main() {
    std::cout << "Starting Assetto Corsa Enhanced SCR Agent...\n";
    
    ACSharedMemory ac;
    if (!ac.init()) {
        std::cerr << "Failed to initialize AC Shared Memory.\n";
        // Continue anyway for testing if AC isn't running
    } else {
        std::cout << "Successfully connected to Assetto Corsa Shared Memory.\n";
    }
    
    VJoyFeeder vjoy(1);
    if (!vjoy.init()) {
        std::cerr << "Failed to initialize vJoy.\n";
    }
    
    SCRController controller;
    controller.init();
    
    ActuatorMapper mapper;
    
    std::cout << "Entering main control loop (100Hz)...\n";
    
    // Main Control Loop
    // AC Physics runs at 333Hz (approx 3ms)
    // We run our loop at 100Hz (10ms)
    while (true) {
        auto* physics = ac.getPhysics();
        if (physics) {
            float target_accel = 0.0f;
            float target_steer = 0.0f;
            
            // 1. Compute optimal targets via MPC
            controller.computeOptimalControl(physics, target_accel, target_steer);
            
            // 2. Map targets to pedal/steering inputs
            float current_speed = physics->speedKmh / 3.6f;
            auto inputs = mapper.mapToPedals(target_accel, target_steer, current_speed);
            
            // 3. Inject inputs to Assetto Corsa via vJoy
            vjoy.sendInputs(inputs.steering_axis, inputs.gas_pedal, inputs.brake_pedal);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return 0;
}
