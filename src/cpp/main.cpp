#if defined(_WIN32)
#define NOGDI
#define NOUSER
#endif

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include "raylib.h"
#include "model/SingleTrack.h"
#include "controller/SCRController.h"

// Simple waypoint structure (compatible with what we'd load from CSV)
struct Waypoint {
    double x, y;
    double inner_bound, outer_bound; 
};

std::vector<Waypoint> generateOvalTrack(double radius, double straight_len, double width, int points) {
    std::vector<Waypoint> track;
    int half_points = points / 2;
    
    // Bottom straight (right to left)
    for (int i = 0; i < straight_len; i += 2) {
        track.push_back({ (double)(straight_len/2 - i), radius, width/2, width/2 });
    }
    
    // Left curve
    for (int i = 0; i <= 180; i += 5) {
        double angle = i * PI / 180.0;
        track.push_back({ -straight_len/2 - radius * std::sin(angle), radius * std::cos(angle), width/2, width/2 });
    }
    
    // Top straight (left to right)
    for (int i = 0; i < straight_len; i += 2) {
        track.push_back({ (double)(-straight_len/2 + i), -radius, width/2, width/2 });
    }
    
    // Right curve
    for (int i = 0; i <= 180; i += 5) {
        double angle = i * PI / 180.0;
        track.push_back({ straight_len/2 + radius * std::sin(angle), -radius * std::cos(angle), width/2, width/2 });
    }
    
    return track;
}

int main() {
    std::cout << "Starting C++ GUI Simulation...\n";
    
    const int screenWidth = 1200;
    const int screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "Enhanced SCR - C++ Simulation");
    SetTargetFPS(60);
    
    // Generate track
    auto track = generateOvalTrack(100.0, 200.0, 10.0, 100);
    
    // Initialize Simulation Model (Ground Truth)
    SingleTrack sim_model;
    Eigen::VectorXd state(6);
    // [x, y, v_long, v_lat, yaw, dyaw]
    state << 0, 100, 5.0, 0, PI, 0; // Start at bottom straight, moving left (yaw=PI), 5 m/s
    
    SCRController controller;
    controller.init();
    
    const double dt = 1.0 / 60.0; // 60Hz physics step
    const double SCALE = 3.0; // pixels per meter
    
    // Camera setup for rendering
    Camera2D camera = { 0 };
    camera.target = { 0, 0 };
    camera.offset = { screenWidth / 2.0f, screenHeight / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    while (!WindowShouldClose()) {
        // --- 1. Control (MPC) ---
        float target_accel = 0.0f;
        float target_steer = 0.0f;
        
        // Convert Eigen state to SPageFilePhysics mock for the controller interface, 
        // OR we can just pass the Eigen state directly for now.
        // For testing the GUI, let's just make it drive straight or use a dummy controller
        
        // Dummy simple controller to follow track (Pure Pursuit roughly)
        // Find closest point
        int closest_idx = 0;
        double min_dist = 1e9;
        for (int i = 0; i < track.size(); i++) {
            double dist = std::hypot(track[i].x - state(0), track[i].y - state(1));
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // Lookahead
        int target_idx = (closest_idx + 10) % track.size();
        double tx = track[target_idx].x;
        double ty = track[target_idx].y;
        
        double dx = tx - state(0);
        double dy = ty - state(1);
        double target_yaw = std::atan2(dy, dx);
        
        double yaw_error = target_yaw - state(4);
        // Normalize yaw error
        while (yaw_error > PI) yaw_error -= 2 * PI;
        while (yaw_error < -PI) yaw_error += 2 * PI;
        
        target_steer = yaw_error * 0.5; // P controller for steering
        
        // Speed control
        if (state(2) < 15.0) target_accel = 2.0;
        else target_accel = -1.0;
        
        // --- 2. Physics Simulation ---
        Eigen::Vector2d u(target_steer, target_accel); // [delta, torque/accel]
        state = sim_model.rk4(state, u, dt);
        
        // Update camera to follow car
        camera.target = { (float)(state(0) * SCALE), (float)(state(1) * SCALE) };

        // --- 3. Rendering ---
        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        BeginMode2D(camera);
        
        // Draw Track
        for (int i = 0; i < track.size(); i++) {
            int next_i = (i + 1) % track.size();
            Vector2 p1 = { (float)(track[i].x * SCALE), (float)(track[i].y * SCALE) };
            Vector2 p2 = { (float)(track[next_i].x * SCALE), (float)(track[next_i].y * SCALE) };
            DrawLineEx(p1, p2, (float)(10.0 * SCALE), LIGHTGRAY);
            DrawLineEx(p1, p2, 1.0f, DARKGRAY); // Centerline
        }
        
        // Draw Car
        Rectangle carRect = { (float)(state(0) * SCALE), (float)(state(1) * SCALE), (float)(4.0 * SCALE), (float)(2.0 * SCALE) };
        Vector2 carOrigin = { carRect.width / 2, carRect.height / 2 };
        DrawRectanglePro(carRect, carOrigin, (float)(state(4) * 180.0 / PI), BLUE);
        
        EndMode2D();
        
        DrawText(TextFormat("Speed: %.1f m/s", state(2)), 10, 10, 20, BLACK);
        DrawText(TextFormat("Steer: %.2f rad", target_steer), 10, 30, 20, BLACK);
        
        EndDrawing();
    }
    
    CloseWindow();
    return 0;
}
