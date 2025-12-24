// #include "Y2Kinematics/QP_solver.hpp"

// /* Convert YMatrix to CSC format for OSQP */ 
// QPSolver::CSCMatrix QPSolver::convertToCSC(const YMatrix& matrix, double tolerance) {
//     CSCMatrix csc;
//     csc.nrows = static_cast<OSQPInt>(matrix.rows());
//     csc.ncols = static_cast<OSQPInt>(matrix.cols());
//     csc.nnz = 0;
    
//     // For P matrix (Hessian), we need only upper triangular part
//     bool is_square = (matrix.rows() == matrix.cols());
    
//     // Count non-zero elements
//     for (size_t j = 0; j < matrix.cols(); ++j) {
//         for (size_t i = 0; i < matrix.rows(); ++i) {
//             // For square matrices (P), only consider upper triangular part
//             if (is_square && i > j) continue;
            
//             if (std::abs(matrix[i][j]) > tolerance) {
//                 csc.nnz++;
//             }
//         }
//     }
    
//     // Reserve space
//     csc.data.reserve(csc.nnz);
//     csc.indices.reserve(csc.nnz);
//     csc.indptr.reserve(csc.ncols + 1);
    
//     // Fill CSC format (column-major)
//     csc.indptr.push_back(0);
//     for (size_t j = 0; j < matrix.cols(); ++j) {
//         for (size_t i = 0; i < matrix.rows(); ++i) {
//             // For square matrices (P), only consider upper triangular part
//             if (is_square && i > j) continue;
            
//             if (std::abs(matrix[i][j]) > tolerance) {
//                 csc.data.push_back(static_cast<OSQPFloat>(matrix[i][j]));
//                 csc.indices.push_back(static_cast<OSQPInt>(i));
//             }
//         }
//         csc.indptr.push_back(static_cast<OSQPInt>(csc.data.size()));
//     }
    
//     return csc;
// }

// /* Convert YMatrix to CSC format for constraint matrices (full matrix) */
// QPSolver::CSCMatrix QPSolver::convertToCSC_full(const YMatrix& matrix, double tolerance) {
//     CSCMatrix csc;
//     csc.nrows = static_cast<OSQPInt>(matrix.rows());
//     csc.ncols = static_cast<OSQPInt>(matrix.cols());
//     csc.nnz = 0;
    
//     // Count non-zero elements (full matrix)
//     for (size_t j = 0; j < matrix.cols(); ++j) {
//         for (size_t i = 0; i < matrix.rows(); ++i) {
//             if (std::abs(matrix[i][j]) > tolerance) {
//                 csc.nnz++;
//             }
//         }
//     }
    
//     // Reserve space
//     csc.data.reserve(csc.nnz);
//     csc.indices.reserve(csc.nnz);
//     csc.indptr.reserve(csc.ncols + 1);
    
//     // Fill CSC format (column-major, full matrix)
//     csc.indptr.push_back(0);
//     for (size_t j = 0; j < matrix.cols(); ++j) {
//         for (size_t i = 0; i < matrix.rows(); ++i) {
//             if (std::abs(matrix[i][j]) > tolerance) {
//                 csc.data.push_back(static_cast<OSQPFloat>(matrix[i][j]));
//                 csc.indices.push_back(static_cast<OSQPInt>(i));
//             }
//         }
//         csc.indptr.push_back(static_cast<OSQPInt>(csc.data.size()));
//     }
    
//     return csc;
// }

// /* Solve QP problem using OSQP */
// QPSolver::QPResult QPSolver::solve(const YMatrix& H, const std::vector<double>& f,
//                 const YMatrix& A_eq, const std::vector<double>& b_eq,
//                 const std::vector<double>& lb, const std::vector<double>& ub,
//                 double tolerance) {
    
//     QPResult result;
//     result.success = false;
    
//     // Input validation
//     if (H.rows() != H.cols()) {
//         std::cerr << "Error: H matrix must be square" << std::endl;
//         return result;
//     }
    
//     if (f.size() != static_cast<size_t>(H.rows())) {
//         std::cerr << "Error: f vector size mismatch with H matrix" << std::endl;
//         return result;
//     }
    
//     if (lb.size() != f.size() || ub.size() != f.size()) {
//         std::cerr << "Error: bound vector size mismatch" << std::endl;
//         return result;
//     }
    
//     if (A_eq.cols() != H.rows()) {
//         std::cerr << "Error: A_eq matrix column size mismatch" << std::endl;
//         return result;
//     }
    
//     if (b_eq.size() != static_cast<size_t>(A_eq.rows())) {
//         std::cerr << "Error: b_eq vector size mismatch with A_eq matrix" << std::endl;
//         return result;
//     }
    
//     OSQPInt n = static_cast<OSQPInt>(H.rows());  // Number of variables
//     OSQPInt m_eq = static_cast<OSQPInt>(A_eq.rows());  // Number of equality constraints
//     OSQPInt m = m_eq + 2*n;  // Total constraints (eq + bounds)
    
//     // Convert matrices to CSC format with validation
//     CSCMatrix P_csc;
//     CSCMatrix A_csc;
    
//     try {
//         // For P matrix (Hessian), convert as upper triangular
//         P_csc = convertToCSC(H);  // convertToCSC now handles upper triangular for square matrices
//         if (P_csc.nnz == 0) {
//             std::cerr << "Warning: P matrix is all zeros, adding small diagonal regularization" << std::endl;
//             YMatrix H_reg = H;
//             for (int i = 0; i < n; ++i) {
//                 H_reg[i][i] += 1e-8;  // Add small regularization
//             }
//             P_csc = convertToCSC(H_reg);
//         }
//     } catch (const std::exception& e) {
//         std::cerr << "Error converting P matrix to CSC: " << e.what() << std::endl;
//         return result;
//     }
    
//     // Create constraint matrix: [A_eq; I; -I] for [equality; lower bounds; upper bounds]
//     YMatrix A_full(m, n);
//     std::vector<OSQPFloat> l_full(m), u_full(m);
    
//     // Add equality constraints
//     for (OSQPInt i = 0; i < m_eq; ++i) {
//         for (OSQPInt j = 0; j < n; ++j) {
//             A_full[i][j] = A_eq[i][j];
//         }
//         l_full[i] = static_cast<OSQPFloat>(b_eq[i]);
//         u_full[i] = static_cast<OSQPFloat>(b_eq[i]);  // Equality: l = u
//     }
    
//     // Add lower bound constraints (I*x >= lb  =>  -I*x <= -lb)
//     for (OSQPInt i = 0; i < n; ++i) {
//         A_full[m_eq + i][i] = -1.0;
//         l_full[m_eq + i] = -OSQP_INFTY;
//         u_full[m_eq + i] = static_cast<OSQPFloat>(-lb[i]);
//     }
    
//     // Add upper bound constraints (I*x <= ub)
//     for (OSQPInt i = 0; i < n; ++i) {
//         A_full[m_eq + n + i][i] = 1.0;
//         l_full[m_eq + n + i] = -OSQP_INFTY;
//         u_full[m_eq + n + i] = static_cast<OSQPFloat>(ub[i]);
//     }
    
//     A_csc = convertToCSC_full(A_full);  // Use full matrix conversion for constraints
    
//     // Validate CSC matrices
//     if (P_csc.data.empty() || A_csc.data.empty()) {
//         std::cerr << "Error: Empty CSC matrix data" << std::endl;
//         return result;
//     }
    
//     // Check for infinite or NaN values
//     for (const auto& val : P_csc.data) {
//         if (!std::isfinite(val)) {
//             std::cerr << "Error: Non-finite value in P matrix" << std::endl;
//             return result;
//         }
//     }
    
//     for (const auto& val : A_csc.data) {
//         if (!std::isfinite(val)) {
//             std::cerr << "Error: Non-finite value in A matrix" << std::endl;
//             return result;
//         }
//     }
    
//     // Convert f vector to OSQPFloat
//     std::vector<OSQPFloat> q_data(f.size());
//     for (size_t i = 0; i < f.size(); ++i) {
//         q_data[i] = static_cast<OSQPFloat>(f[i]);
//     }
    
//     // Create CSC matrices using new OSQP API
//     OSQPCscMatrix* P = OSQPCscMatrix_new(P_csc.nrows, P_csc.ncols, P_csc.nnz,
//                                          P_csc.data.data(), P_csc.indices.data(), P_csc.indptr.data());
//     OSQPCscMatrix* A = OSQPCscMatrix_new(A_csc.nrows, A_csc.ncols, A_csc.nnz,
//                                          A_csc.data.data(), A_csc.indices.data(), A_csc.indptr.data());
    
//     if (!P || !A) {
//         std::cerr << "Error: Failed to create CSC matrices" << std::endl;
//         if (P) OSQPCscMatrix_free(P);
//         if (A) OSQPCscMatrix_free(A);
//         return result;
//     }
    
//     // Setup settings
//     OSQPSettings* settings = OSQPSettings_new();
//     if (!settings) {
//         std::cerr << "Error: Failed to create OSQP settings" << std::endl;
//         OSQPCscMatrix_free(P);
//         OSQPCscMatrix_free(A);
//         return result;
//     }
    
//     settings->eps_abs = static_cast<OSQPFloat>(tolerance);
//     settings->eps_rel = static_cast<OSQPFloat>(tolerance);
//     settings->verbose = 0;  // Set to 1 for debug output
//     settings->max_iter = 20000;
    
//     // Setup OSQP solver using new API
//     OSQPSolver* solver = nullptr;
//     OSQPInt exitflag = osqp_setup(&solver, P, q_data.data(), A, l_full.data(), u_full.data(), m, n, settings);
    
//     if (exitflag != 0 || !solver) {
//         std::cerr << "Error: OSQP setup failed with exitflag " << exitflag << std::endl;
//         OSQPCscMatrix_free(P);
//         OSQPCscMatrix_free(A);
//         OSQPSettings_free(settings);
//         return result;
//     }
    
//     // Solve problem
//     exitflag = osqp_solve(solver);
    
//     if (exitflag == 0) {
//         // Try to get solution using direct access (most common in Ubuntu 22.04)
//         if (solver->solution && solver->solution->x) {
//             if (solver->info) {
//                 result.success = (solver->info->status_val == OSQP_SOLVED || 
//                                 solver->info->status_val == OSQP_SOLVED_INACCURATE);
//                 result.status = solver->info->status_val;
//                 result.objective_value = solver->info->obj_val;
//             }
            
//             if (result.success) {
//                 result.solution.resize(n);
//                 for (OSQPInt i = 0; i < n; ++i) {
//                     result.solution[i] = static_cast<double>(solver->solution->x[i]);
//                 }
//             }
//         } else {
//             // Try newer API with output parameter
//             OSQPSolution solution_out;
//             OSQPInt get_result = osqp_get_solution(solver, &solution_out);
//             if (get_result == 0 && solution_out.x) {
//                 // Note: This might not have info available
//                 result.success = true;
//                 result.status = OSQP_SOLVED;
//                 result.objective_value = 0.0;  // May not be available
                
//                 result.solution.resize(n);
//                 for (OSQPInt i = 0; i < n; ++i) {
//                     result.solution[i] = static_cast<double>(solution_out.x[i]);
//                 }
//             } else {
//                 std::cerr << "Error: Failed to get solution, get_result: " << get_result << std::endl;
//             }
//         }
//     } else {
//         std::cerr << "Error: OSQP solve failed with exitflag " << exitflag << std::endl;
//     }
    
//     // Cleanup
//     osqp_cleanup(solver);
//     OSQPCscMatrix_free(P);
//     OSQPCscMatrix_free(A);
//     OSQPSettings_free(settings);
    
//     return result;
// }

#include "Y2Kinematics/QP_solver.hpp"

/* Convert YMatrix to CSC format for OSQP */ 
QPSolver::CSCMatrix QPSolver::convertToCSC(const YMatrix& matrix, double tolerance) {
    CSCMatrix csc;
    csc.nrows = static_cast<OSQPInt>(matrix.rows());
    csc.ncols = static_cast<OSQPInt>(matrix.cols());
    csc.nnz = 0;
    
    // For P matrix (Hessian), we need only upper triangular part
    bool is_square = (matrix.rows() == matrix.cols());
    
    // Count non-zero elements
    for (size_t j = 0; j < matrix.cols(); ++j) {
        for (size_t i = 0; i < matrix.rows(); ++i) {
            if (is_square && i > j) continue;
            if (std::abs(matrix[i][j]) > tolerance) csc.nnz++;
        }
    }
    
    // Reserve space
    csc.data.reserve(csc.nnz);
    csc.indices.reserve(csc.nnz);
    csc.indptr.reserve(csc.ncols + 1);
    
    // Fill CSC format (column-major)
    csc.indptr.push_back(0);
    for (size_t j = 0; j < matrix.cols(); ++j) {
        for (size_t i = 0; i < matrix.rows(); ++i) {
            if (is_square && i > j) continue;
            if (std::abs(matrix[i][j]) > tolerance) {
                csc.data.push_back(static_cast<OSQPFloat>(matrix[i][j]));
                csc.indices.push_back(static_cast<OSQPInt>(i));
            }
        }
        csc.indptr.push_back(static_cast<OSQPInt>(csc.data.size()));
    }
    
    return csc;
}

/* Convert YMatrix to CSC format for constraint matrices (full matrix) */
QPSolver::CSCMatrix QPSolver::convertToCSC_full(const YMatrix& matrix, double tolerance) {
    CSCMatrix csc;
    csc.nrows = static_cast<OSQPInt>(matrix.rows());
    csc.ncols = static_cast<OSQPInt>(matrix.cols());
    csc.nnz = 0;
    
    // Count non-zero elements (full matrix)
    for (size_t j = 0; j < matrix.cols(); ++j) {
        for (size_t i = 0; i < matrix.rows(); ++i) {
            if (std::abs(matrix[i][j]) > tolerance) csc.nnz++;
        }
    }
    
    // Reserve space
    csc.data.reserve(csc.nnz);
    csc.indices.reserve(csc.nnz);
    csc.indptr.reserve(csc.ncols + 1);
    
    // Fill CSC format (column-major, full matrix)
    csc.indptr.push_back(0);
    for (size_t j = 0; j < matrix.cols(); ++j) {
        for (size_t i = 0; i < matrix.rows(); ++i) {
            if (std::abs(matrix[i][j]) > tolerance) {
                csc.data.push_back(static_cast<OSQPFloat>(matrix[i][j]));
                csc.indices.push_back(static_cast<OSQPInt>(i));
            }
        }
        csc.indptr.push_back(static_cast<OSQPInt>(csc.data.size()));
    }
    
    return csc;
}

/* Solve QP problem using OSQP (warm-start included) */
QPSolver::QPResult QPSolver::solve(const YMatrix& H, const std::vector<double>& f,
                const YMatrix& A_eq, const std::vector<double>& b_eq,
                const std::vector<double>& lb, const std::vector<double>& ub,
                double tolerance) {
    
    QPResult result;
    result.success = false;
    
    // Input validation
    if (H.rows() != H.cols()) {
        std::cerr << "Error: H matrix must be square" << std::endl;
        return result;
    }
    if (f.size() != static_cast<size_t>(H.rows())) {
        std::cerr << "Error: f vector size mismatch with H matrix" << std::endl;
        return result;
    }
    if (lb.size() != f.size() || ub.size() != f.size()) {
        std::cerr << "Error: bound vector size mismatch" << std::endl;
        return result;
    }
    if (A_eq.cols() != H.rows()) {
        std::cerr << "Error: A_eq matrix column size mismatch" << std::endl;
        return result;
    }
    if (b_eq.size() != static_cast<size_t>(A_eq.rows())) {
        std::cerr << "Error: b_eq vector size mismatch with A_eq matrix" << std::endl;
        return result;
    }
    
    OSQPInt n    = static_cast<OSQPInt>(H.rows());   // variables
    OSQPInt m_eq = static_cast<OSQPInt>(A_eq.rows());  // equalities
    OSQPInt m    = m_eq + 2*n;                       // total constraints
    
    // Convert matrices to CSC
    CSCMatrix P_csc;
    CSCMatrix A_csc;
    try {
        P_csc = convertToCSC(H);
        if (P_csc.nnz == 0) {
            std::cerr << "Warning: P is zero; adding tiny diagonal\n";
            YMatrix H_reg = H;
            for (int i = 0; i < n; ++i) H_reg[i][i] += 1e-8;
            P_csc = convertToCSC(H_reg);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error converting P to CSC: " << e.what() << std::endl;
        return result;
    }
    
    // A_full = [ A_eq ; -I ; I ]
    YMatrix A_full(m, n);
    std::vector<OSQPFloat> l_full(m), u_full(m);
    
    // equalities
    for (OSQPInt i = 0; i < m_eq; ++i) {
        for (OSQPInt j = 0; j < n; ++j) A_full[i][j] = A_eq[i][j];
        l_full[i] = static_cast<OSQPFloat>(b_eq[i]);
        u_full[i] = static_cast<OSQPFloat>(b_eq[i]);  // l = u
    }
    // lower bounds: -I x <= -lb
    for (OSQPInt i = 0; i < n; ++i) {
        A_full[m_eq + i][i] = -1.0;
        l_full[m_eq + i]    = -OSQP_INFTY;
        u_full[m_eq + i]    = static_cast<OSQPFloat>(-lb[i]);
    }
    // upper bounds: I x <= ub
    for (OSQPInt i = 0; i < n; ++i) {
        A_full[m_eq + n + i][i] = 1.0;
        l_full[m_eq + n + i]    = -OSQP_INFTY;
        u_full[m_eq + n + i]    = static_cast<OSQPFloat>(ub[i]);
    }
    
    A_csc = convertToCSC_full(A_full);
    if (P_csc.data.empty() || A_csc.data.empty()) {
        std::cerr << "Error: Empty CSC data" << std::endl;
        return result;
    }
    for (const auto& v : P_csc.data) if (!std::isfinite(v)) { std::cerr << "Non-finite in P\n"; return result; }
    for (const auto& v : A_csc.data) if (!std::isfinite(v)) { std::cerr << "Non-finite in A\n"; return result; }
    
    // linear term q
    std::vector<OSQPFloat> q_data(f.size());
    for (size_t i = 0; i < f.size(); ++i) q_data[i] = static_cast<OSQPFloat>(f[i]);
    
    // Build OSQP matrices
    OSQPCscMatrix* P = OSQPCscMatrix_new(P_csc.nrows, P_csc.ncols, P_csc.nnz,
                                         P_csc.data.data(), P_csc.indices.data(), P_csc.indptr.data());
    OSQPCscMatrix* A = OSQPCscMatrix_new(A_csc.nrows, A_csc.ncols, A_csc.nnz,
                                         A_csc.data.data(), A_csc.indices.data(), A_csc.indptr.data());
    if (!P || !A) {
        std::cerr << "Error: Failed to create CSC matrices" << std::endl;
        if (P) OSQPCscMatrix_free(P);
        if (A) OSQPCscMatrix_free(A);
        return result;
    }
    
    // Settings
    OSQPSettings* settings = OSQPSettings_new();
    if (!settings) {
        std::cerr << "Error: Failed to create settings" << std::endl;
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A);
        return result;
    }
    settings->eps_abs  = static_cast<OSQPFloat>(tolerance);
    settings->eps_rel  = static_cast<OSQPFloat>(tolerance);
    settings->verbose  = 0;
    settings->max_iter = 20000;
    // settings->polish = 1;
    
    // Setup
    OSQPSolver* solver = nullptr;
    OSQPInt exitflag = osqp_setup(&solver, P, q_data.data(), A, l_full.data(), u_full.data(), m, n, settings);
    if (exitflag != 0 || !solver) {
        std::cerr << "Error: osqp_setup failed, exitflag=" << exitflag << std::endl;
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A);
        OSQPSettings_free(settings);
        return result;
    }
    
    // -------- Warm start (OSQP 구버전 API: osqp_warm_start) --------
    if (has_warm_ && last_n_ == n && last_m_ == m) {
        const OSQPFloat* x0 = warm_x_.empty() ? nullptr : warm_x_.data();
        const OSQPFloat* y0 = (warm_y_.size() == static_cast<size_t>(m)) ? warm_y_.data() : nullptr;
        OSQPInt rc = osqp_warm_start(solver, x0, y0);
        (void)rc; // 실패해도 치명적 아님
    } else {
        has_warm_ = false;
        warm_x_.clear();
        warm_y_.clear();
    }
    
    // Solve
    exitflag = osqp_solve(solver);
    
    if (exitflag == 0) {
        if (solver->solution && solver->solution->x) {
            if (solver->info) {
                result.success = (solver->info->status_val == OSQP_SOLVED ||
                                  solver->info->status_val == OSQP_SOLVED_INACCURATE);
                result.status = solver->info->status_val;
                result.objective_value = solver->info->obj_val;
            }
            if (result.success) {
                result.solution.resize(n);
                for (OSQPInt i = 0; i < n; ++i)
                    result.solution[i] = static_cast<double>(solver->solution->x[i]);

                // cache for next warm-start
                warm_x_.assign(solver->solution->x, solver->solution->x + n);
                if (solver->solution->y) {
                    warm_y_.assign(solver->solution->y, solver->solution->y + m);
                } else {
                    warm_y_.assign(static_cast<size_t>(m), static_cast<OSQPFloat>(0.0));
                }
                has_warm_ = true;
                last_n_ = n;
                last_m_ = m;
            } else {
                clearWarmStart();
            }
        } else {
            // Fallback for other API variants
            OSQPSolution solution_out;
            OSQPInt get_result = osqp_get_solution(solver, &solution_out);
            if (get_result == 0 && solution_out.x) {
                result.success = true;
                result.status  = OSQP_SOLVED;
                result.objective_value = 0.0;
                result.solution.resize(n);
                for (OSQPInt i = 0; i < n; ++i)
                    result.solution[i] = static_cast<double>(solution_out.x[i]);

                warm_x_.assign(solution_out.x, solution_out.x + n);
                warm_y_.assign(static_cast<size_t>(m), static_cast<OSQPFloat>(0.0));
                has_warm_ = true;
                last_n_ = n;
                last_m_ = m;
            } else {
                clearWarmStart();
                std::cerr << "Error: Failed to get solution, get_result=" << get_result << std::endl;
            }
        }
    } else {
        std::cerr << "Error: osqp_solve failed, exitflag=" << exitflag << std::endl;
        clearWarmStart();
    }
    
    // Cleanup
    osqp_cleanup(solver);
    OSQPCscMatrix_free(P);
    OSQPCscMatrix_free(A);
    OSQPSettings_free(settings);
    
    return result;
}
