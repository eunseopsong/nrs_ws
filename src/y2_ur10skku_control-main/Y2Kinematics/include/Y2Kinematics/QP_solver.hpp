// #pragma once
// #include <osqp/osqp.h>
// #include "Y2Matrix/YMatrix.hpp"
// #include <vector>
// #include <iostream>
// #include <memory>
// #include <cstring>

// class QPSolver {
// private:
//     // Convert YMatrix to CSC (Compressed Sparse Column) format for OSQP
//     struct CSCMatrix {
//         std::vector<OSQPFloat> data;
//         std::vector<OSQPInt> indices;
//         std::vector<OSQPInt> indptr;
//         OSQPInt nrows, ncols, nnz;
//     };
    
//     CSCMatrix convertToCSC(const YMatrix& matrix, double tolerance = 1e-12);
//     CSCMatrix convertToCSC_full(const YMatrix& matrix, double tolerance = 1e-12);  // For constraint matrices

// public:
//     struct QPResult {
//         std::vector<double> solution;
//         bool success;
//         OSQPInt status;
//         OSQPFloat objective_value;
//     };
    
//     QPResult solve(const YMatrix& H, const std::vector<double>& f,
//                    const YMatrix& A_eq, const std::vector<double>& b_eq,
//                    const std::vector<double>& lb, const std::vector<double>& ub,
//                    double tolerance = 1e-4);
// };


#pragma once
#include <osqp/osqp.h>
#include "Y2Matrix/YMatrix.hpp"
#include <vector>
#include <iostream>
#include <memory>
#include <cstring>
#include <cmath>
#include <algorithm>

class QPSolver {
private:
    // Convert YMatrix to CSC (Compressed Sparse Column) format for OSQP
    struct CSCMatrix {
        std::vector<OSQPFloat> data;
        std::vector<OSQPInt>   indices;
        std::vector<OSQPInt>   indptr;
        OSQPInt nrows = 0, ncols = 0, nnz = 0;
    };
    
    CSCMatrix convertToCSC(const YMatrix& matrix, double tolerance = 1e-12);
    CSCMatrix convertToCSC_full(const YMatrix& matrix, double tolerance = 1e-12);  // For constraint matrices

    // ---- Warm-start buffers ----
    std::vector<OSQPFloat> warm_x_;   // previous primal solution
    std::vector<OSQPFloat> warm_y_;   // previous dual solution (length = m)
    bool    has_warm_ = false;
    OSQPInt last_n_   = 0;
    OSQPInt last_m_   = 0;

public:
    struct QPResult {
        std::vector<double> solution;
        bool      success = false;
        OSQPInt   status  = 0;
        OSQPFloat objective_value = 0.0;
    };

    QPSolver() = default;

    void setWarmStart(const std::vector<double>& x0,
                      const std::vector<double>& y0 = std::vector<double>()) {
        warm_x_.assign(x0.begin(), x0.end());
        warm_y_.assign(y0.begin(), y0.end());
        has_warm_ = !warm_x_.empty();
        last_n_ = static_cast<OSQPInt>(warm_x_.size());
        last_m_ = static_cast<OSQPInt>(warm_y_.size());
    }

    void clearWarmStart() {
        warm_x_.clear();
        warm_y_.clear();
        has_warm_ = false;
        last_n_ = 0;
        last_m_ = 0;
    }
    
    QPResult solve(const YMatrix& H, const std::vector<double>& f,
                   const YMatrix& A_eq, const std::vector<double>& b_eq,
                   const std::vector<double>& lb, const std::vector<double>& ub,
                   double tolerance = 1e-4);
};
