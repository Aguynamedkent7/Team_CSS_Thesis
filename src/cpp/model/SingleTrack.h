#pragma once
#include <Eigen/Dense>

class SingleTrack {
public:
    struct Parameters {
        double m = 1500.0;    // Mass (kg)
        double Iz = 2500.0;   // Yaw inertia
        double l_f = 1.3;     // Dist to front axle
        double l_r = 1.4;     // Dist to rear axle
        
        // Pacejka
        double Bf = 10.0, Cf = 1.9, Df = 7350.0; 
        double Br = 10.0, Cr = 1.9, Dr = 7350.0;
        
        // Drivetrain
        double Cm1 = 1500.0, Cm2 = 0.0, Cr0 = 0.0, Cr2 = 0.0;
    };

    SingleTrack(Parameters p = Parameters());

    // States: p_x, p_y, v_long, v_lat, yaw, dyaw (6)
    // Inputs: delta, torque (2)
    Eigen::VectorXd ode(const Eigen::VectorXd& x, const Eigen::Vector2d& u);

    Eigen::VectorXd rk4(const Eigen::VectorXd& x, const Eigen::Vector2d& u, double dt);
    
    void getLinearizedDynamics(const Eigen::VectorXd& x, const Eigen::Vector2d& u, double dt, 
                               Eigen::MatrixXd& A, Eigen::MatrixXd& B);

private:
    Parameters p;
};
