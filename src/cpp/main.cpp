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

std::vector<Waypoint> generateHockenheim(double scale) {
    std::vector<Waypoint> track;
    double track_width = 0.25 * scale;
    
    double x = 0.0, y = 0.0, yaw = 0.0;
    
    auto add_turn = [&](double angle_deg, double length_m, int segments) {
        double d_theta = (angle_deg * PI / 180.0) / segments;
        double ds = (length_m * scale) / segments;
        
        for (int i = 0; i < segments; i++) {
            if (std::abs(d_theta) < 1e-6) {
                x += ds * std::cos(yaw);
                y += ds * std::sin(yaw);
            } else {
                double R = (length_m * scale) / (angle_deg * PI / 180.0);
                x += R * (std::sin(yaw + d_theta) - std::sin(yaw));
                y -= R * (std::cos(yaw + d_theta) - std::cos(yaw));
                yaw += d_theta;
            }
            track.push_back({ x, y, track_width/2.0, track_width/2.0 });
        }
    };
    
    // section 1
    add_turn(0, 4.4, 50);
    add_turn(-75, 75/360.0 * 0.9702 * PI, 30);
    add_turn(0, 2.01, 50);
    
    // section 2
    add_turn(-105, 105/360.0 * 0.5958 * PI, 30);
    add_turn(0, 0.2943, 50);
    add_turn(-30, 30/360.0 * 0.5958 * PI, 30);
    add_turn(0, 0.6381, 50);
    add_turn(45, 45/360.0 * 1.3228 * PI, 30);
    add_turn(0, 1.9571, 50);
    add_turn(45, 45/360.0 * 2.2246 * PI, 30);
    add_turn(0, 0.5332, 50);
    add_turn(-30, 30/360.0 * 1.5899 * PI, 30);
    add_turn(0, 0.2704, 50);
    add_turn(-105, 105/360.0 * 0.7923 * PI, 30);
    
    // section 3
    add_turn(0, 0.8, 50);
    add_turn(-90, 90/360.0 * 1.4027 * PI, 30);
    add_turn(0, 1.7915, 50);
    add_turn(135, 135/360.0 * 0.6866 * PI, 30);
    add_turn(0, 0.6, 50);
    add_turn(75, 75/360.0 * 1.0 * PI, 30);
    add_turn(0, 0.2146, 50);
    add_turn(-30, 30/360.0 * 0.8 * PI, 30);
    add_turn(0, 0.5829, 50);
    add_turn(-75, 75/360.0 * 0.7 * PI, 30);
    add_turn(0, 0.5439, 50);
    add_turn(-120, 105/360.0 * 1.1826 * PI, 30);
    add_turn(0, 0.5, 50);
    
    return track;
}

int main() {
    std::cout << "Starting C++ GUI Simulation...\n";
    
    const int screenWidth = 1200;
    const int screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "Enhanced SCR - C++ Simulation");
    SetTargetFPS(60);
    
    // Generate track (1:1 scale by multiplying 1:43 scale track by 43)
    auto track = generateHockenheim(43.0);
    
    // Initialize Simulation Model (Ground Truth)
    SingleTrack sim_model;
    Eigen::VectorXd state(6);
    // [x, y, v_long, v_lat, yaw, dyaw]
    state << track[0].x, track[0].y, 5.0, 0, 0, 0; // Start at first point
    
    // Build TrackPoints for Controller
    std::vector<TrackPoint> mpc_track;
    for (size_t i = 0; i < track.size(); ++i) {
        size_t next_i = (i + 1) % track.size();
        Eigen::Vector2d current(track[i].x, track[i].y);
        Eigen::Vector2d next(track[next_i].x, track[next_i].y);
        
        Eigen::Vector2d forward = (next - current).normalized();
        Eigen::Vector2d normal(-forward(1), forward(0)); // Points left
        
        TrackPoint tp;
        tp.center = current;
        tp.forward = forward;
        tp.normal = normal;
        tp.left_dist = track[i].inner_bound;
        tp.right_dist = track[i].outer_bound;
        mpc_track.push_back(tp);
    }
    
    const double dt = 1.0 / 60.0; // 60Hz physics step
    const double SCALE = 3.0; // pixels per meter
    
    ControllerParams mpc_params;
    mpc_params.Hp = 20;
    mpc_params.dt = dt;
    mpc_params.mode = TrackConstraintMode::SL;
    
    SCRController controller;
    controller.init(mpc_params, mpc_track);
    
    // Camera setup for rendering
    Camera2D camera = { 0 };
    camera.target = { 0, 0 };
    camera.offset = { screenWidth / 2.0f, screenHeight / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    int prev_closest_idx = 0;
    double start_time = GetTime();
    double last_lap_time = 0.0;
    double best_lap_time = 0.0;

    while (!WindowShouldClose()) {
        // --- 1. Control (MPC) ---
        float target_accel = 0.0f;
        float target_steer = 0.0f;
        
        // Find closest point for Lap Timing
        int closest_idx = 0;
        double min_dist = 1e9;
        for (int i = 0; i < track.size(); i++) {
            double dist = std::hypot(track[i].x - state(0), track[i].y - state(1));
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // Lap timing logic
        if (closest_idx < track.size() * 0.1 && prev_closest_idx > track.size() * 0.9) {
            last_lap_time = GetTime() - start_time;
            if (best_lap_time == 0.0 || last_lap_time < best_lap_time) {
                best_lap_time = last_lap_time;
            }
            start_time = GetTime();
        }
        prev_closest_idx = closest_idx;
        
        double current_lap_time = GetTime() - start_time;
        double delta = (best_lap_time > 0.0) ? (current_lap_time - best_lap_time) : 0.0;
        
        // RUN LTV-MPC
        controller.computeOptimalControl(state, target_accel, target_steer);
        
        // Clamp steer (safety override)
        if (target_steer > 0.2) target_steer = 0.2;
        if (target_steer < -0.2) target_steer = -0.2;
        
        // --- 2. Physics Simulation ---
        Eigen::Vector2d u(target_steer, target_accel); // [delta, torque/accel]
        state = sim_model.rk4(state, u, dt);
        
        // Update camera to follow car (Negate Y for Raylib's top-left origin)
        camera.target = { (float)(state(0) * SCALE), (float)(-state(1) * SCALE) };

        // --- 3. Rendering ---
        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        BeginMode2D(camera);
        
        // Draw Track
        for (int i = 0; i < track.size(); i++) {
            int next_i = (i + 1) % track.size();
            Vector2 p1 = { (float)(track[i].x * SCALE), (float)(-track[i].y * SCALE) };
            Vector2 p2 = { (float)(track[next_i].x * SCALE), (float)(-track[next_i].y * SCALE) };
            DrawLineEx(p1, p2, (float)(track[i].inner_bound * 2.0 * SCALE), LIGHTGRAY);
            DrawLineEx(p1, p2, 1.0f, DARKGRAY); // Centerline
        }
        
        // Draw lookahead target (MPC Trajectory)
        if (controller.getPredictedTrajectory().cols() > 0) {
            const Eigen::MatrixXd& pred = controller.getPredictedTrajectory();
            for (int k = 0; k < pred.cols(); ++k) {
                DrawCircle((int)(pred(0, k) * SCALE), (int)(-pred(1, k) * SCALE), 2.0f, RED);
            }
        }
        
        // Draw Car
        Rectangle carRect = { (float)(state(0) * SCALE), (float)(-state(1) * SCALE), (float)(4.0 * SCALE), (float)(2.0 * SCALE) };
        Vector2 carOrigin = { carRect.width / 2, carRect.height / 2 };
        DrawRectanglePro(carRect, carOrigin, (float)(-state(4) * 180.0 / PI), BLUE);
        
        EndMode2D();
        
        // HUD
        DrawText(TextFormat("Speed: %.1f m/s (%.1f km/h)", state(2), state(2)*3.6), 10, 10, 20, BLACK);
        DrawText(TextFormat("Steer: %.2f rad", target_steer), 10, 30, 20, BLACK);
        DrawText(TextFormat("Lap Time: %.2f s", current_lap_time), 10, 60, 20, DARKBLUE);
        DrawText(TextFormat("Last Lap: %.2f s", last_lap_time), 10, 80, 20, GRAY);
        DrawText(TextFormat("Best Lap: %.2f s", best_lap_time), 10, 100, 20, GOLD);
        DrawText(TextFormat("Delta: %+.2f s", delta), 10, 120, 20, delta > 0 ? RED : GREEN);
        
        // Pedals HUD
        DrawText("Throttle", 10, 160, 20, DARKGRAY);
        DrawRectangle(100, 160, (target_accel > 0 ? (target_accel / 4.0) * 100.0 : 0), 20, GREEN);
        DrawRectangleLines(100, 160, 100, 20, BLACK);
        
        DrawText("Brake", 10, 190, 20, DARKGRAY);
        DrawRectangle(100, 190, (target_accel < 0 ? (-target_accel / 5.0) * 100.0 : 0), 20, RED);
        DrawRectangleLines(100, 190, 100, 20, BLACK);
        
        EndDrawing();
    }
    
    CloseWindow();
    return 0;
}
