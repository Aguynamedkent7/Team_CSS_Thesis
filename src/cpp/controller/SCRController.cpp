#include "SCRController.h"
#include "../model/SingleTrack.h"
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>
#include <cmath>

SCRController::SCRController() {
    m_solver = std::make_unique<OsqpEigen::Solver>();
    std::cout << "[SCRController] Constructed\n";
}

SCRController::~SCRController() = default;

void SCRController::init(const ControllerParams& params, const std::vector<TrackPoint>& track) {
    m_params = params;
    m_track = track;
    
    // Initialize OSQP settings
    m_solver->settings()->setVerbosity(false);
    m_solver->settings()->setWarmStart(true);
    m_solver->settings()->setAbsoluteTolerance(1e-4);
    m_solver->settings()->setRelativeTolerance(1e-4);
    
    m_initialized = false;
}

void SCRController::warmStart(const Eigen::VectorXd& current_state) {
    int Hp = m_params.Hp;
    X_opt_prev = Eigen::MatrixXd::Zero(6, Hp);
    U_opt_prev = Eigen::MatrixXd::Zero(2, Hp);
    
    SingleTrack model;
    Eigen::VectorXd x = current_state;
    Eigen::Vector2d u(0.0, 0.0); // Coasting straight
    
    for (int k = 0; k < Hp; ++k) {
        x = model.rk4(x, u, m_params.dt);
        X_opt_prev.col(k) = x;
        U_opt_prev.col(k) = u;
    }
}

void SCRController::formulateQP(const Eigen::VectorXd& current_state) {
    int Hp = m_params.Hp;
    int nx = 6;
    int nu = 2;
    int ns = nx + nu; // variables per step
    int n_vars = ns * Hp + 1; // +1 for slack
    
    // Constraints:
    // 1. Dynamics: x_{k+1} = A x_k + B u_k + d (nx * Hp)
    // 2. Track bounds (SL): left/right limits (2 * Hp)
    int n_eq = nx * Hp;
    int n_ineq = 2 * Hp; 
    if (m_params.mode == TrackConstraintMode::SCR) {
        // Just placeholder for now, assume 4 sides per polygon
        n_ineq = 4 * Hp; 
    }
    int n_cons = n_eq + n_ineq;
    
    std::vector<Eigen::Triplet<double>> H_triplets;
    std::vector<Eigen::Triplet<double>> A_triplets;
    Eigen::VectorXd f = Eigen::VectorXd::Zero(n_vars);
    Eigen::VectorXd l = Eigen::VectorXd::Constant(n_cons, -OsqpEigen::INFTY);
    Eigen::VectorXd u = Eigen::VectorXd::Constant(n_cons, OsqpEigen::INFTY);
    
    SingleTrack model;
    
    Eigen::VectorXd x_k = current_state;
    
    // Construct Objective and Dynamics Constraints
    int eq_row = 0;
    int ineq_row = n_eq;
    int slack_idx = n_vars - 1;
    
    for (int k = 0; k < Hp; ++k) {
        int idx_xk = k * ns;       // start of state vars for step k
        int idx_uk = k * ns + nx;  // start of input vars for step k
        
        // 1. Linearize Dynamics around previous trajectory
        Eigen::Vector2d u_prev = U_opt_prev.col(k);
        Eigen::MatrixXd Ad, Bd;
        model.getLinearizedDynamics(x_k, u_prev, m_params.dt, Ad, Bd);
        
        // Compute affine term: d = f(x_k, u_prev) - Ad*x_k - Bd*u_prev
        Eigen::VectorXd x_next_exact = model.rk4(x_k, u_prev, m_params.dt);
        Eigen::VectorXd d = x_next_exact - Ad * x_k - Bd * u_prev;
        
        // Dynamics Constraint: x_{k+1} - A_d x_k - B_d u_k = d
        // For OSQP: A_eq * z = b_eq
        for (int i = 0; i < nx; ++i) {
            A_triplets.push_back(Eigen::Triplet<double>(eq_row + i, idx_xk + i, 1.0)); // x_{k+1}
            for (int j = 0; j < nu; ++j) {
                A_triplets.push_back(Eigen::Triplet<double>(eq_row + i, idx_uk + j, -Bd(i, j)));
            }
            if (k > 0) {
                int idx_x_prev = (k - 1) * ns;
                for (int j = 0; j < nx; ++j) {
                    A_triplets.push_back(Eigen::Triplet<double>(eq_row + i, idx_x_prev + j, -Ad(i, j)));
                }
            }
            l(eq_row + i) = d(i);
            u(eq_row + i) = d(i);
        }
        if (k == 0) {
            // First step x_0 is fixed to current_state
            Eigen::VectorXd d0 = d + Ad * current_state;
            for(int i = 0; i < nx; ++i) {
                l(eq_row + i) = d0(i);
                u(eq_row + i) = d0(i);
            }
        }
        
        // 2. Objective: Maximize track progress
        // We find the closest track point to the PREVIOUS predicted position
        int closest_cp = 0;
        double min_dist = 1e9;
        Eigen::Vector2d pos_prev(X_opt_prev(0, k), X_opt_prev(1, k));
        
        for (size_t i = 0; i < m_track.size(); ++i) {
            double dist = (m_track[i].center - pos_prev).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_cp = i;
            }
        }
        
        const auto& cp = m_track[closest_cp];
        
        // Maximize progress: -Q * (forward_vector \cdot position)
        if (k == Hp - 1) { // Only on terminal state or all states? MATLAB does terminal state for SL
            f(idx_xk + 0) = -m_params.Q_pos * cp.forward(0);
            f(idx_xk + 1) = -m_params.Q_pos * cp.forward(1);
        }
        
        // 3. Track Bounds
        if (m_params.mode == TrackConstraintMode::SL) {
            // normal_vector * position - slack <= normal_vector * left_bound
            // -normal_vector * position - slack <= -normal_vector * right_bound
            
            // Left constraint
            A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 0, cp.normal(0)));
            A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 1, cp.normal(1)));
            A_triplets.push_back(Eigen::Triplet<double>(ineq_row, slack_idx, -1.0));
            u(ineq_row) = cp.normal.dot(cp.center + cp.normal * cp.left_dist);
            l(ineq_row) = -OsqpEigen::INFTY;
            ineq_row++;
            
            // Right constraint
            A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 0, -cp.normal(0)));
            A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 1, -cp.normal(1)));
            A_triplets.push_back(Eigen::Triplet<double>(ineq_row, slack_idx, -1.0));
            u(ineq_row) = -cp.normal.dot(cp.center - cp.normal * cp.right_dist);
            l(ineq_row) = -OsqpEigen::INFTY;
            ineq_row++;
        } else if (m_params.mode == TrackConstraintMode::SCR || m_params.mode == TrackConstraintMode::ENHANCED_SCR) {
            // Convex Polygon Constraints (A * pos <= b)
            // Assumes cp.scr_polygon is populated with CCW vertices
            if (!cp.scr_polygon.empty()) {
                size_t num_verts = cp.scr_polygon.size();
                for (size_t v = 0; v < num_verts; ++v) {
                    Eigen::Vector2d p1 = cp.scr_polygon[v];
                    Eigen::Vector2d p2 = cp.scr_polygon[(v + 1) % num_verts];
                    
                    Eigen::Vector2d edge = p2 - p1;
                    Eigen::Vector2d inward_normal(-edge(1), edge(0)); // Rotate 90 deg CCW
                    inward_normal.normalize();
                    
                    // We want: inward_normal \cdot (pos - p1) >= 0
                    // Which is: -inward_normal \cdot pos <= -inward_normal \cdot p1
                    
                    A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 0, -inward_normal(0)));
                    A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 1, -inward_normal(1)));
                    A_triplets.push_back(Eigen::Triplet<double>(ineq_row, slack_idx, -1.0)); // Slack
                    
                    u(ineq_row) = -inward_normal.dot(p1);
                    l(ineq_row) = -OsqpEigen::INFTY;
                    ineq_row++;
                }
            } else {
                // Fallback to SL if polygon is missing
                A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 0, cp.normal(0)));
                A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 1, cp.normal(1)));
                A_triplets.push_back(Eigen::Triplet<double>(ineq_row, slack_idx, -1.0));
                u(ineq_row) = cp.normal.dot(cp.center + cp.normal * cp.left_dist);
                l(ineq_row) = -OsqpEigen::INFTY;
                ineq_row++;
                
                A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 0, -cp.normal(0)));
                A_triplets.push_back(Eigen::Triplet<double>(ineq_row, idx_xk + 1, -cp.normal(1)));
                A_triplets.push_back(Eigen::Triplet<double>(ineq_row, slack_idx, -1.0));
                u(ineq_row) = -cp.normal.dot(cp.center - cp.normal * cp.right_dist);
                l(ineq_row) = -OsqpEigen::INFTY;
                ineq_row++;
            }
        }
        
        // Input limits via bounding box inequalities (or OSQP bounds)
        // We can just add them to the constraint matrix
        // Wait, OSQP supports variable bounds directly! We can just use A_triplets for identity.
        // Actually, OSQP l <= Ax <= u. We can just add identity rows for inputs.
        // For simplicity, we add rows to A.
        
        // Update x_k for next linearization
        x_k = x_next_exact;
        eq_row += nx;
    }
    
    // Add Slack variable cost
    f(slack_idx) = m_params.S_slack;
    
    // Add Input delta cost (R)
    for (int k = 0; k < Hp - 1; ++k) {
        int uk = k * ns + nx;
        int uk_next = (k + 1) * ns + nx;
        
        H_triplets.push_back(Eigen::Triplet<double>(uk + 0, uk + 0, 2 * m_params.R(0,0)));
        H_triplets.push_back(Eigen::Triplet<double>(uk + 1, uk + 1, 2 * m_params.R(1,1)));
        
        H_triplets.push_back(Eigen::Triplet<double>(uk + 0, uk_next + 0, -m_params.R(0,0)));
        H_triplets.push_back(Eigen::Triplet<double>(uk_next + 0, uk + 0, -m_params.R(0,0)));
        
        H_triplets.push_back(Eigen::Triplet<double>(uk + 1, uk_next + 1, -m_params.R(1,1)));
        H_triplets.push_back(Eigen::Triplet<double>(uk_next + 1, uk + 1, -m_params.R(1,1)));
    }
    
    // First and last input delta cost fixes
    H_triplets.push_back(Eigen::Triplet<double>(nx, nx, m_params.R(0,0)));
    H_triplets.push_back(Eigen::Triplet<double>(nx+1, nx+1, m_params.R(1,1)));
    
    int last_u = (Hp - 1) * ns + nx;
    H_triplets.push_back(Eigen::Triplet<double>(last_u, last_u, m_params.R(0,0)));
    H_triplets.push_back(Eigen::Triplet<double>(last_u+1, last_u+1, m_params.R(1,1)));
    
    Eigen::SparseMatrix<double> H(n_vars, n_vars);
    H.setFromTriplets(H_triplets.begin(), H_triplets.end());
    
    Eigen::SparseMatrix<double> A(n_cons, n_vars);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());
    
    // Setup OSQP
    m_solver->data()->clearLinearConstraintsMatrix();
    m_solver->data()->clearHessianMatrix();
    
    if (!m_solver->isInitialized()) {
        m_solver->data()->setNumberOfVariables(n_vars);
        m_solver->data()->setNumberOfConstraints(n_cons);
        m_solver->data()->setHessianMatrix(H);
        m_solver->data()->setGradient(f);
        m_solver->data()->setLinearConstraintsMatrix(A);
        m_solver->data()->setLowerBound(l);
        m_solver->data()->setUpperBound(u);
        m_solver->initSolver();
    } else {
        m_solver->updateHessianMatrix(H);
        m_solver->updateGradient(f);
        m_solver->updateLinearConstraintsMatrix(A);
        m_solver->updateBounds(l, u);
    }
}

void SCRController::computeOptimalControl(const Eigen::VectorXd& current_state, float& out_accel, float& out_steer) {
    if (m_track.empty()) return;
    
    if (!m_initialized) {
        warmStart(current_state);
        m_params.R << 10.0, 0, 0, 1.0; // Weights for [steer, accel]
        m_initialized = true;
    }
    
    formulateQP(current_state);
    
    if (m_solver->solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        Eigen::VectorXd QPSolution = m_solver->getSolution();
        
        int nx = 6;
        int nu = 2;
        int ns = nx + nu;
        
        for (int k = 0; k < m_params.Hp; ++k) {
            int idx_xk = k * ns;
            int idx_uk = k * ns + nx;
            
            X_opt_prev.col(k) = QPSolution.segment(idx_xk, nx);
            U_opt_prev.col(k) = QPSolution.segment(idx_uk, nu);
        }
        
        out_steer = U_opt_prev(0, 0);
        out_accel = U_opt_prev(1, 0);
    } else {
        std::cout << "[SCRController] QP Failed! Using warm start fallback.\n";
        out_steer = U_opt_prev(0, 0);
        out_accel = U_opt_prev(1, 0);
    }
}
