#include <iostream>
#include <vector>
#include <osqp/osqp.h>

int main() {
    std::cout << "Testing OSQP installation and API..." << std::endl;
    
    // Simple 2x2 QP problem: min 0.5 * x^T * P * x + q^T * x
    // subject to: l <= A * x <= u
    
    // Problem data: minimize (x1-1)^2 + (x2-0.5)^2
    // P = [[2, 0], [0, 2]]
    // q = [-2, -1]
    // A = [[1, 1]]  (x1 + x2 <= 1)
    // l = [-inf], u = [1]
    
    OSQPInt n = 2;  // Number of variables
    OSQPInt m = 1;  // Number of constraints
    
    // P matrix (upper triangular): 2x2 identity * 2
    OSQPFloat P_x[2] = {2.0, 2.0};
    OSQPInt P_i[2] = {0, 1};
    OSQPInt P_p[3] = {0, 1, 2};
    
    // Linear term
    OSQPFloat q[2] = {-2.0, -1.0};
    
    // Constraint matrix A: [1, 1]
    OSQPFloat A_x[2] = {1.0, 1.0};
    OSQPInt A_i[2] = {0, 0};
    OSQPInt A_p[3] = {0, 1, 2};
    
    // Constraint bounds
    OSQPFloat l[1] = {-OSQP_INFTY};
    OSQPFloat u[1] = {1.0};
    
    std::cout << "Creating CSC matrices..." << std::endl;
    
    // Create CSC matrices
    OSQPCscMatrix* P = OSQPCscMatrix_new(n, n, 2, P_x, P_i, P_p);
    OSQPCscMatrix* A = OSQPCscMatrix_new(m, n, 2, A_x, A_i, A_p);
    
    if (!P || !A) {
        std::cerr << "Failed to create CSC matrices" << std::endl;
        if (P) OSQPCscMatrix_free(P);
        if (A) OSQPCscMatrix_free(A);
        return -1;
    }
    
    std::cout << "Creating settings..." << std::endl;
    
    // Create settings
    OSQPSettings* settings = OSQPSettings_new();
    if (!settings) {
        std::cerr << "Failed to create settings" << std::endl;
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A);
        return -1;
    }
    
    settings->verbose = 1;
    settings->eps_abs = 1e-3;
    settings->eps_rel = 1e-3;
    
    std::cout << "Setting up solver..." << std::endl;
    
    // Setup solver
    OSQPSolver* solver = nullptr;
    OSQPInt exitflag = osqp_setup(&solver, P, q, A, l, u, m, n, settings);
    
    std::cout << "Setup exitflag: " << exitflag << std::endl;
    
    if (exitflag != 0 || !solver) {
        std::cerr << "Setup failed with exitflag: " << exitflag << std::endl;
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A);
        OSQPSettings_free(settings);
        return -1;
    }
    
    std::cout << "Solving problem..." << std::endl;
    
    // Solve
    exitflag = osqp_solve(solver);
    
    std::cout << "Solve exitflag: " << exitflag << std::endl;
    
    if (exitflag == 0) {
        // Try different ways to get solution based on API version
        
        // Method 1: Direct access (older API)
        if (solver->solution && solver->solution->x) {
            std::cout << "Solution found (direct access)!" << std::endl;
            if (solver->info) {
                std::cout << "Status: " << solver->info->status_val << std::endl;
                std::cout << "Iterations: " << solver->info->iter << std::endl;
                std::cout << "Objective: " << solver->info->obj_val << std::endl;
            }
            std::cout << "Solution: [" << solver->solution->x[0] << ", " << solver->solution->x[1] << "]" << std::endl;
        }
        // Method 2: Try newer API with output parameter
        else {
            OSQPSolution solution_out;
            OSQPInt get_result = osqp_get_solution(solver, &solution_out);
            if (get_result == 0 && solution_out.x) {
                std::cout << "Solution found (output parameter)!" << std::endl;
                std::cout << "Solution: [" << solution_out.x[0] << ", " << solution_out.x[1] << "]" << std::endl;
            } else {
                std::cerr << "Failed to get solution, get_result: " << get_result << std::endl;
            }
        }
    } else {
        std::cerr << "Solve failed with exitflag: " << exitflag << std::endl;
    }
    
    // Cleanup
    osqp_cleanup(solver);
    OSQPCscMatrix_free(P);
    OSQPCscMatrix_free(A);
    OSQPSettings_free(settings);
    
    std::cout << "Test completed." << std::endl;
    return 0;
}