#pragma once
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include "../ac_structs.h"

class SCRController {
public:
    SCRController();
    void init();
    
    // Runs the Sequential Convex Programming loop (Double-Resolution)
    void computeOptimalControl(const SPageFilePhysics* state, float& out_accel, float& out_steer);

private:
    void formulateQP(const Eigen::VectorXd& current_state);
    void calculateDynamicTerminalConstraint();

    OsqpEigen::Solver m_solver;
};
