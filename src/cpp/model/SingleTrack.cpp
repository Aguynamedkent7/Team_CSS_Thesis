#include "SingleTrack.h"
#include <cmath>

SingleTrack::SingleTrack(Parameters p) : p(p) {}

Eigen::VectorXd SingleTrack::ode(const Eigen::VectorXd& x, const Eigen::Vector2d& u) {
    // States
    // x(0) = p_x
    // x(1) = p_y
    // x(2) = v_long
    // x(3) = v_lat
    // x(4) = yaw
    // x(5) = dyaw
    
    double v_long = x(2);
    double v_lat  = x(3);
    double yaw    = x(4);
    double dyaw   = x(5);
    
    // Inputs
    double delta  = u(0);
    double t      = u(1);
    
    // Avoid division by zero
    if (std::abs(v_long) < 0.1) {
        v_long = (v_long >= 0) ? 0.1 : -0.1;
    }

    // Tire forces
    double alpha_f = -std::atan2(dyaw * p.l_f + v_lat, v_long) + delta;
    double alpha_r =  std::atan2(dyaw * p.l_r - v_lat, v_long);
    
    double F_fy = p.Df * std::sin(p.Cf * std::atan(p.Bf * alpha_f));
    double F_ry = p.Dr * std::sin(p.Cr * std::atan(p.Br * alpha_r));
    
    double F_rx = (p.Cm1 - p.Cm2 * v_long) * t - p.Cr0 - p.Cr2 * v_long * v_long;
    
    Eigen::VectorXd dX(6);
    dX(0) = v_long * std::cos(yaw) - v_lat * std::sin(yaw);
    dX(1) = v_long * std::sin(yaw) + v_lat * std::cos(yaw);
    dX(2) = 1.0 / p.m * (F_rx - F_fy * std::sin(delta) + p.m * v_lat * dyaw);
    dX(3) = 1.0 / p.m * (F_ry + F_fy * std::cos(delta) - p.m * v_long * dyaw);
    dX(4) = dyaw;
    dX(5) = 1.0 / p.Iz * (F_fy * p.l_f * std::cos(delta) - F_ry * p.l_r);
    
    return dX;
}

Eigen::VectorXd SingleTrack::rk4(const Eigen::VectorXd& x, const Eigen::Vector2d& u, double dt) {
    Eigen::VectorXd k1 = ode(x, u);
    Eigen::VectorXd k2 = ode(x + 0.5 * dt * k1, u);
    Eigen::VectorXd k3 = ode(x + 0.5 * dt * k2, u);
    Eigen::VectorXd k4 = ode(x + dt * k3, u);
    
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

void SingleTrack::getLinearizedDynamics(const Eigen::VectorXd& x, const Eigen::Vector2d& u, double dt, 
                                        Eigen::MatrixXd& A, Eigen::MatrixXd& B) {
    const double epsilon = 1e-5;
    
    A.resize(x.size(), x.size());
    B.resize(x.size(), u.size());
    
    Eigen::VectorXd x_next = rk4(x, u, dt);
    
    // State Jacobian (A)
    for (int i = 0; i < x.size(); ++i) {
        Eigen::VectorXd x_plus = x;
        x_plus(i) += epsilon;
        A.col(i) = (rk4(x_plus, u, dt) - x_next) / epsilon;
    }
    
    // Input Jacobian (B)
    for (int i = 0; i < u.size(); ++i) {
        Eigen::Vector2d u_plus = u;
        u_plus(i) += epsilon;
        B.col(i) = (rk4(x, u_plus, dt) - x_next) / epsilon;
    }
}
