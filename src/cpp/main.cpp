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
    
    using namespace std::chrono;
    const int TARGET_HZ = 100;
    const nanoseconds target_duration(1000000000 / TARGET_HZ); // 10ms
    
    while (true) {
        auto loop_start = high_resolution_clock::now();
        
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
        
        auto loop_end = high_resolution_clock::now();
        auto elapsed = loop_end - loop_start;
        
        if (elapsed < target_duration) {
            std::this_thread::sleep_for(target_duration - elapsed);
        } else {
            std::cerr << "WARNING: Solver missed 100Hz deadline! Took " 
                      << duration_cast<milliseconds>(elapsed).count() << "ms\n";
        }
    }
    
    return 0;
}
