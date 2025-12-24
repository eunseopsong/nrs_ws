#if 1 // cost - function q^2

#include "Y2Kinematics/Kinematics.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <limits>

// Constructor for Kinematics class
Kinematics::Kinematics(double SamplingTime_, size_t numOfAxis_, const YMatrix& EE2TCP_)
:dt(SamplingTime_), numOfAxis(numOfAxis_), EE2TCP(EE2TCP_) {
    // Joint limits
    q_min.assign(numOfAxis,-2*M_PI);
    q_max.assign(numOfAxis, 2*M_PI);
    qd_min.assign(numOfAxis,(q_min[0]/2)/dt);
    qd_max.assign(numOfAxis,(q_max[0]/2)/dt);

    // Accel limits default: wide (no effective constraint)
    a_min.assign(numOfAxis, -std::numeric_limits<double>::infinity());
    a_max.assign(numOfAxis,  std::numeric_limits<double>::infinity());

    // prev q uninitialized yet
    q_prev.resize(numOfAxis, 0.0);
    has_prev = false;
}

// Enhanced QP-based IK solver
std::vector<double> Kinematics::solve_IK(const std::vector<double>& q_current, 
                                         const YMatrix& target_HTM_) {
    
    // 0. Transform target HTM to match EE2TCP
    auto target_HTM = target_HTM_*EE2TCP.inverse();

    // 1. Calculate current end-effector pose
    YMatrix current_HTM = forwardKinematics(q_current);
    current_HTM = current_HTM*EE2TCP.inverse();

    /* 정밀도 제거 */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            current_HTM[i][j] = roundToNthDecimal(current_HTM[i][j], QP_precision);
            target_HTM[i][j]  = roundToNthDecimal(target_HTM[i][j],  QP_precision);
        }
    }
    
    // 2. Calculate pose errors
    std::vector<double> e_pos(3);
    for (int i = 0; i < 3; i++) {
        e_pos[i] = target_HTM[i][3] - current_HTM[i][3];
    }
    
    YMatrix R_target = target_HTM.extract(0, 0, 3, 3);
    YMatrix R_current = current_HTM.extract(0, 0, 3, 3);
    YMatrix R_err = R_target * R_current.transpose();
    
    std::vector<double> e_rot(3);
    e_rot[0] = 0.5 * (R_err[2][1] - R_err[1][2]);
    e_rot[1] = 0.5 * (R_err[0][2] - R_err[2][0]);
    e_rot[2] = 0.5 * (R_err[1][0] - R_err[0][1]);
    
    std::vector<double> Delta_x_des(6);
    for (int i = 0; i < 3; i++) {
        Delta_x_des[i]   = Kp_pos * e_pos[i];
        Delta_x_des[i+3] = Kp_rot * e_rot[i];
    }
    
    // 3. Calculate Jacobian
    YMatrix J = calculateJacobian(q_current);
    
    // 4. Setup QP matrices
    YMatrix I   = YMatrix::identity(numOfAxis);
    YMatrix JtJ = J.transpose() * J;
    
    // H = 2*omega_p*I + 2*alpha*(J'*J + lambda*I)^{-1}   // 사용자가 기존에 두신 형태 유지
    YMatrix H1 = I * (2.0 * omega_p);

    YMatrix J_metric     = JtJ + I * lambda;     // SPD
    YMatrix J_metric_inv = J_metric.inverse();   // (권장: Cholesky)
    YMatrix H2 = J_metric_inv * (2.0 * alpha);
    
    YMatrix H = H1 + H2;
    
    // f = -2*omega_p*q_current
    std::vector<double> f(numOfAxis);
    for (int i = 0; i < (int)numOfAxis; i++) {
        f[i] = -2.0 * omega_p * q_current[i];
    }
    
    // Equality constraint: J*q_next = Delta_x_des + J*q_current
    YMatrix q_matrix(numOfAxis, 1);
    for (int i = 0; i < (int)numOfAxis; i++) q_matrix[i][0] = q_current[i];
    YMatrix Jq = J * q_matrix;
    
    std::vector<double> beq(6);
    for (int i = 0; i < 6; i++) beq[i] = Delta_x_des[i] + Jq[i][0];
    
    // 5. Bounds assembly (joint angle, velocity, and ACCELERATION)
    std::vector<double> lb(numOfAxis), ub(numOfAxis);
    for (int i = 0; i < (int)numOfAxis; i++) {
        // angle limits
        double lb_angle = q_min[i];
        double ub_angle = q_max[i];

        // velocity limits -> q_next \in [ q_current + dt*qd_min,  q_current + dt*qd_max ]
        double lb_vel = q_current[i] + dt * qd_min[i];
        double ub_vel = q_current[i] + dt * qd_max[i];

        // acceleration limits (if q_prev is known)
        // q_next >= dt^2*a_min + 2*q_current - q_prev
        // q_next <= dt^2*a_max + 2*q_current - q_prev
        double lb_acc = -std::numeric_limits<double>::infinity();
        double ub_acc =  std::numeric_limits<double>::infinity();

        if (has_prev) {
            lb_acc = dt*dt * a_min[i] + 2.0 * q_current[i] - q_prev[i];
            ub_acc = dt*dt * a_max[i] + 2.0 * q_current[i] - q_prev[i];
        }

        // intersect all bounds
        lb[i] = std::max({lb_angle, lb_vel, lb_acc});
        ub[i] = std::min({ub_angle, ub_vel, ub_acc});

        // (optional) sanity: if infeasible, widen slightly or report
        if (lb[i] > ub[i]) {
            // 간단 처리: 가속 경계를 우선 완화 (a_min/a_max 가 너무 타이트한 경우)
            // 현실 시스템에서는 사용자 로그/알림 또는 가중치/목표 수정 권장
            double mid = 0.5 * (lb[i] + ub[i]);
            lb[i] = mid - 1e-9;
            ub[i] = mid + 1e-9;
        }
    }
    
    // 6. Solve QP
    QPSolver::QPResult result = qp_solver.solve(H, f, J, beq, lb, ub, QP_tolerance);
    
    if (!result.success) {
        std::cerr << "Warning: QP solver failed with status " << result.status << std::endl;
        return q_current;  // Return current angles if failed
    }

    // 7. Update history for next step (q_prev <- q_current)
    q_prev = q_current;
    has_prev = true;
    
    return result.solution;
}

// Utility methods
double Kinematics::roundToNthDecimal(double value, int n) {
    double multiplier = std::pow(10.0, n);
    return std::round(value * multiplier) / multiplier;
}

void Kinematics::setControlGains(double kp_pos, double kp_rot) {
    Kp_pos = kp_pos;
    Kp_rot = kp_rot;
}

void Kinematics::setQPWeights(double omega_p_val, double alpha_val, double lambda_val) {
    omega_p = omega_p_val;
    alpha   = alpha_val;
    lambda  = lambda_val;
} 

void Kinematics::setJointLimits(const std::vector<double>& q_min_in, 
                                const std::vector<double>& q_max_in,
                                const std::vector<double>& qd_min_in, 
                                const std::vector<double>& qd_max_in) {
    if (q_min_in.size() != numOfAxis || q_max_in.size() != numOfAxis ||
        qd_min_in.size() != numOfAxis || qd_max_in.size() != numOfAxis) {
        throw std::invalid_argument("Joint limits must have " + std::to_string(numOfAxis) + " elements each");
    }
    q_min = q_min_in;
    q_max = q_max_in;
    qd_min = qd_min_in;
    qd_max = qd_max_in;
}

void Kinematics::setAccelLimits(const std::vector<double>& a_min_in,
                                const std::vector<double>& a_max_in) {
    if (a_min_in.size() != numOfAxis || a_max_in.size() != numOfAxis) {
        throw std::invalid_argument("Acceleration limits must have " + std::to_string(numOfAxis) + " elements each");
    }
    a_min = a_min_in;
    a_max = a_max_in;
}

void Kinematics::setPrevQ(const std::vector<double>& q_prev_in) {
    if (q_prev_in.size() != numOfAxis) {
        throw std::invalid_argument("q_prev must have " + std::to_string(numOfAxis) + " elements");
    }
    q_prev = q_prev_in;
    has_prev = true;
}

// Print pose in a readable format
void Kinematics::printPose(const YMatrix& pose, const std::string& label) {
    std::cout << label << " Pose:\n";
    for (const auto& row : pose) {
        for (const auto& val : row) {
            std::cout << val << " ";
        }
        std::cout << "\n";
    }
}
#endif

#if 0 // cost - function jq-v
#include "Y2Kinematics/Kinematics.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <limits>

// ---- small helpers (internal) ----
namespace {
inline void ensure(bool cond, const char* msg){
    if(!cond) throw std::runtime_error(msg);
}
inline bool isFiniteMatrix(const YMatrix& M){
    for(size_t i=0;i<M.rows();++i)
        for(size_t j=0;j<M.cols();++j)
            if(!std::isfinite(M[i][j])) return false;
    return true;
}
}

// Constructor
Kinematics::Kinematics(double SamplingTime_, size_t numOfAxis_, const YMatrix& EE2TCP_)
: dt(SamplingTime_), numOfAxis(numOfAxis_), EE2TCP(EE2TCP_) {
    // Joint limits (wide default)
    q_min.assign(numOfAxis, -2.0 * M_PI);
    q_max.assign(numOfAxis,  2.0 * M_PI);
    qd_min.assign(numOfAxis, (q_min[0] / 2.0) / dt);
    qd_max.assign(numOfAxis, (q_max[0] / 2.0) / dt);

    // Accel limits default: unbounded
    a_min.assign(numOfAxis, -std::numeric_limits<double>::infinity());
    a_max.assign(numOfAxis,  std::numeric_limits<double>::infinity());

    q_prev.resize(numOfAxis, 0.0);
    has_prev = false;
}

// QP IK (Δq formulation):
//   min  1/2 || J Δq - Δx_des ||_W^2 + 1/2 λ ||Δq||^2
//   s.t. q_min - q_k ≤ Δq ≤ q_max - q_k
//        qd_min dt   ≤ Δq ≤ qd_max dt
//        (opt) dt^2 a_min + q_k - q_{k-1} ≤ Δq ≤ dt^2 a_max + q_k - q_{k-1}
std::vector<double> Kinematics::solve_IK(const std::vector<double>& q_current, 
                                         const YMatrix& target_HTM_input) 
{
    const int n = static_cast<int>(numOfAxis);
    ensure((int)q_current.size() == n, "solve_IK: q_current size mismatch");

    // 0) Align TCP frame
    YMatrix target_HTM = target_HTM_input * EE2TCP.inverse();

    // 1) Current EE pose
    YMatrix current_HTM = forwardKinematics(q_current) * EE2TCP.inverse();

    ensure(current_HTM.rows()==4 && current_HTM.cols()==4, "solve_IK: current_HTM must be 4x4");
    ensure(target_HTM.rows()==4 && target_HTM.cols()==4, "solve_IK: target_HTM must be 4x4");

    // (Optional) numeric rounding for stability/log readability
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            current_HTM[i][j] = roundToNthDecimal(current_HTM[i][j], QP_precision);
            target_HTM[i][j]  = roundToNthDecimal(target_HTM[i][j],  QP_precision);
        }
    }

    // 2) Pose error → Δx_des (6×1)
    // Position error
    std::vector<double> e_pos(3);
    for (int i = 0; i < 3; ++i)
        e_pos[i] = target_HTM[i][3] - current_HTM[i][3];

    // Orientation error (small-angle approx from R_err)
    YMatrix R_t = target_HTM.extract(0, 0, 3, 3);
    YMatrix R_c = current_HTM.extract(0, 0, 3, 3);
    YMatrix R_err = R_t * R_c.transpose();

    std::vector<double> e_rot(3);
    e_rot[0] = 0.5 * (R_err[2][1] - R_err[1][2]);
    e_rot[1] = 0.5 * (R_err[0][2] - R_err[2][0]);
    e_rot[2] = 0.5 * (R_err[1][0] - R_err[0][1]);

    // Desired task change (acts like EE twist target)
    std::vector<double> Delta_x_des(6);
    for (int i = 0; i < 3; ++i) {
        Delta_x_des[i]   = Kp_pos * e_pos[i];
        Delta_x_des[i+3] = Kp_rot * e_rot[i];
    }

    // 3) Jacobian (6×n)
    YMatrix J = calculateJacobian(q_current);
    ensure(J.rows()==6 && (int)J.cols()==n, "solve_IK: Jacobian must be 6×n");

    // 4) Build QP cost: H = Jᵀ W J + λ I,  f = - Jᵀ W Δx_des
    YMatrix I = YMatrix::identity(n);

    // W (6×6 diagonal): position vs orientation weights
    YMatrix W = YMatrix::identity(6);
    for (int i = 0; i < 3; ++i) W[i][i] = omega_p; // position
    for (int i = 3; i < 6; ++i) W[i][i] = alpha;   // orientation

    // dx column vector
    YMatrix dx(6, 1);
    for (int i = 0; i < 6; ++i) dx[i][0] = Delta_x_des[i];

    // H (n×n), ensure SPD via λ
    YMatrix H = J.transpose() * (W * J) + I * lambda;
    ensure(H.rows()==(size_t)n && H.cols()==(size_t)n, "solve_IK: H size error");

    // f = -Jᵀ W dx
    YMatrix f_mat = (J.transpose() * (W * dx)) * (-1.0);
    std::vector<double> f(numOfAxis, 0.0);
    for (int i = 0; i < n; ++i) f[i] = f_mat[i][0];

    ensure(isFiniteMatrix(H), "solve_IK: H has non-finite values");
    for(double v: f) ensure(std::isfinite(v), "solve_IK: f has non-finite values");

    // 5) Bounds for Δq
    std::vector<double> lb(n), ub(n);
    for (int i = 0; i < n; ++i) {
        // angle bounds: q_min - q_k ≤ Δq ≤ q_max - q_k
        double lb_angle = q_min[i] - q_current[i];
        double ub_angle = q_max[i] - q_current[i];

        // velocity bounds: qd_min*dt ≤ Δq ≤ qd_max*dt
        double lb_vel = dt * qd_min[i];
        double ub_vel = dt * qd_max[i];

        // (optional) acceleration bounds if q_{k-1} known
        double lb_acc = -std::numeric_limits<double>::infinity();
        double ub_acc =  std::numeric_limits<double>::infinity();
        if (has_prev) {
            lb_acc = dt*dt * a_min[i] + q_current[i] - q_prev[i];
            ub_acc = dt*dt * a_max[i] + q_current[i] - q_prev[i];
        }

        lb[i] = std::max({lb_angle, lb_vel, lb_acc});
        ub[i] = std::min({ub_angle, ub_vel, ub_acc});

        if (lb[i] > ub[i]) {
            // Narrow infeasible box slightly around midpoint to proceed
            double mid = 0.5 * (lb[i] + ub[i]);
            lb[i] = mid - 1e-9;
            ub[i] = mid + 1e-9;
        }
    }

    // 6) SAFE: put a 1×n dummy equality (0 = 0) instead of 0-row matrix to avoid segfaults
    YMatrix Aeq(1, n);  // all zeros
    for(int j=0;j<n;++j) Aeq[0][j] = 0.0;
    std::vector<double> beq(1, 0.0);

    // 7) Solve QP for Δq
    QPSolver::QPResult result = qp_solver.solve(H, f, Aeq, beq, lb, ub, QP_tolerance);
    if (!result.success) {
        std::cerr << "Warning: QP solver failed with status " << result.status << std::endl;
        return q_current; // keep current on failure
    }
    ensure((int)result.solution.size()==n, "solve_IK: solver returned wrong-sized solution");

    // 8) q_next = q_current + Δq
    std::vector<double> q_next(numOfAxis);
    for (int i = 0; i < n; ++i) {
        double dqi = result.solution[i];
        ensure(std::isfinite(dqi), "solve_IK: solution has non-finite entry");
        q_next[i] = q_current[i] + dqi;
        // clamp to hard joint limits
        q_next[i] = std::min(std::max(q_next[i], q_min[i]), q_max[i]);
    }

    // 9) history
    q_prev = q_current;
    has_prev = true;

    return q_next;
}

// Utils
double Kinematics::roundToNthDecimal(double value, int n) {
    double m = std::pow(10.0, n);
    return std::round(value * m) / m;
}

void Kinematics::setControlGains(double kp_pos, double kp_rot) {
    Kp_pos = kp_pos;
    Kp_rot = kp_rot;
}

void Kinematics::setQPWeights(double omega_p_val, double alpha_val, double lambda_val) {
    omega_p = omega_p_val; // position weight in W
    alpha   = alpha_val;   // orientation weight in W
    lambda  = lambda_val;  // damping
}

void Kinematics::setJointLimits(const std::vector<double>& q_min_in, 
                                const std::vector<double>& q_max_in,
                                const std::vector<double>& qd_min_in, 
                                const std::vector<double>& qd_max_in) {
    if (q_min_in.size() != numOfAxis || q_max_in.size() != numOfAxis ||
        qd_min_in.size() != numOfAxis || qd_max_in.size() != numOfAxis) {
        throw std::invalid_argument("Joint limits must have correct dimension");
    }
    q_min = q_min_in;
    q_max = q_max_in;
    qd_min = qd_min_in;
    qd_max = qd_max_in;
}

void Kinematics::setAccelLimits(const std::vector<double>& a_min_in,
                                const std::vector<double>& a_max_in) {
    if (a_min_in.size() != numOfAxis || a_max_in.size() != numOfAxis) {
        throw std::invalid_argument("Acceleration limits must have correct dimension");
    }
    a_min = a_min_in;
    a_max = a_max_in;
}

void Kinematics::setPrevQ(const std::vector<double>& q_prev_in) {
    if (q_prev_in.size() != numOfAxis) {
        throw std::invalid_argument("q_prev must have correct dimension");
    }
    q_prev = q_prev_in;
    has_prev = true;
}

void Kinematics::printPose(const YMatrix& pose, const std::string& label) {
    std::cout << label << " Pose:\n";
    for (const auto& row : pose) {
        for (const auto& val : row) std::cout << val << " ";
        std::cout << "\n";
    }
}
#endif