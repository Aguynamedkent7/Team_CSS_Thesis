#pragma once
// #include <Eigen/Dense>
// #include <OsqpEigen/OsqpEigen.h>
#include "../ac_structs.h"

// Note: Eigen and OsqpEigen includes are commented out to allow basic project compilation
// before installing the libraries.

class SCRController {
public:
    SCRController();
    void init();
    
    // Runs the Sequential Convex Programming loop (Double-Resolution)
    void computeOptimalControl(const SPageFilePhysics* state, float& out_accel, float& out_steer);

private:
    // void formulateQP(const Eigen::VectorXd& current_state);
    void calculateDynamicTerminalConstraint();

    // OsqpEigen::Solver m_solver;
};
