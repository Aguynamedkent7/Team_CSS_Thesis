#pragma once
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "../ac_structs.h"

namespace OsqpEigen {
    class Solver;
}

enum class TrackConstraintMode {
    SL,         // Sequential Linearization (left/right distance)
    SCR,        // Sequential Convex Restriction (polygons)
    ENHANCED_SCR// Improved SCR
};

struct ControllerParams {
    int Hp = 30;              // Prediction horizon
    double dt = 0.05;         // Discretization time (20Hz controller)
    
    // Weights
    double Q_pos = 10.0;      // Track progress maximization
    double Q_vel = 1.0;       // Terminal velocity tie-breaker
    double W_alpha = 10.0;    // Slip angle penalty
    double S_slack = 1000.0;  // Slack penalty for track limits
    
    Eigen::Matrix2d R;        // Control change penalty
    
    // Limits
    Eigen::Vector2d u_min = {-0.5, -10.0}; // [steer, accel]
    Eigen::Vector2d u_max = { 0.5,  5.0};
    
    TrackConstraintMode mode = TrackConstraintMode::SL;
};

// Represents a track point
struct TrackPoint {
    Eigen::Vector2d center;
    Eigen::Vector2d forward;
    Eigen::Vector2d normal; // Points left
    double left_dist;
    double right_dist;
    std::vector<Eigen::Vector2d> scr_polygon; // For SCR convex bounds
};

class SCRController {
public:
    SCRController();
    ~SCRController();
    void init(const ControllerParams& params, const std::vector<TrackPoint>& track);
    
    // Runs the Sequential Convex Programming loop
    void computeOptimalControl(const Eigen::VectorXd& current_state, float& out_accel, float& out_steer);
    
    const Eigen::MatrixXd& getPredictedTrajectory() const { return X_opt_prev; }

private:
    void formulateQP(const Eigen::VectorXd& current_state);
    void extractSolution();
    void warmStart(const Eigen::VectorXd& current_state);

    std::unique_ptr<OsqpEigen::Solver> m_solver;
    ControllerParams m_params;
    std::vector<TrackPoint> m_track;
    
    // Previous optimal trajectories for SQP linearization
    Eigen::MatrixXd X_opt_prev; // 6 x Hp
    Eigen::MatrixXd U_opt_prev; // 2 x Hp
    
    bool m_initialized = false;
};
