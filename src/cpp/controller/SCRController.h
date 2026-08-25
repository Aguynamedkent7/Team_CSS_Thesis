#pragma once
#include <Eigen/Dense>
#include <memory>
#include "../ac_structs.h"

namespace OsqpEigen {
    class Solver;
}

class SCRController {
public:
    SCRController();
    ~SCRController();
    void init();
    
    // Runs the Sequential Convex Programming loop (Double-Resolution)
    void computeOptimalControl(const SPageFilePhysics* state, float& out_accel, float& out_steer);

private:
    void formulateQP(const Eigen::VectorXd& current_state);
    void calculateDynamicTerminalConstraint();

    std::unique_ptr<OsqpEigen::Solver> m_solver;
};
