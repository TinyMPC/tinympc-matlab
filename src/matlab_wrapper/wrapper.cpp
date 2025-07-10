#include "mex.h"
#include "matrix.h"
#include <iostream>
#include <cstring>
#include <memory>

// Fix the Eigen include issue by using the standard approach
#include <Eigen/Dense>
#include <Eigen/Core>

// Now include TinyMPC headers (after Eigen is properly included)
#include "tinympc/tiny_api.hpp"
#include "tinympc/types.hpp"

// Global solver pointer - in a full implementation, this should be managed better
static std::unique_ptr<TinySolver> g_solver = nullptr;

// Helper function to convert MATLAB array to Eigen matrix
Eigen::MatrixXd matlab_to_eigen(const mxArray* mx_array) {
    if (!mxIsDouble(mx_array) || mxIsComplex(mx_array)) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "Input must be a real double array");
    }
    
    size_t rows = mxGetM(mx_array);
    size_t cols = mxGetN(mx_array);
    double* data = mxGetPr(mx_array);
    
    return Eigen::Map<Eigen::MatrixXd>(data, rows, cols);
}

// Helper function to convert Eigen matrix to MATLAB array
mxArray* eigen_to_matlab(const Eigen::MatrixXd& eigen_mat) {
    size_t rows = eigen_mat.rows();
    size_t cols = eigen_mat.cols();
    
    mxArray* mx_array = mxCreateDoubleMatrix(rows, cols, mxREAL);
    double* data = mxGetPr(mx_array);
    
    // Copy data (Eigen is column-major, MATLAB is column-major, so direct copy)
    std::memcpy(data, eigen_mat.data(), rows * cols * sizeof(double));
    
    return mx_array;
}

// Setup function - initialize the solver
void setup_solver(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected arguments: A, B, fdyn, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, verbose
    if (nrhs != 15) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "Setup requires 14 input arguments");
    }
    
    // Extract matrices
    auto A = matlab_to_eigen(prhs[1]);
    auto B = matlab_to_eigen(prhs[2]);
    auto fdyn = matlab_to_eigen(prhs[3]);
    auto Q = matlab_to_eigen(prhs[4]);
    auto R = matlab_to_eigen(prhs[5]);
    
    // Extract scalar parameters
    double rho = mxGetScalar(prhs[6]);
    int nx = (int)mxGetScalar(prhs[7]);
    int nu = (int)mxGetScalar(prhs[8]);
    int N = (int)mxGetScalar(prhs[9]);
    
    // Extract bound matrices
    auto x_min = matlab_to_eigen(prhs[10]);
    auto x_max = matlab_to_eigen(prhs[11]);
    auto u_min = matlab_to_eigen(prhs[12]);
    auto u_max = matlab_to_eigen(prhs[13]);
    
    int verbose = (int)mxGetScalar(prhs[14]);
    
    if (verbose) {
        mexPrintf("Setting up TinyMPC solver with nx=%d, nu=%d, N=%d, rho=%f\n", nx, nu, N, rho);
    }
    
    try {
        // Create solver
        TinySolver* solver_ptr = nullptr;
        
        // Convert Eigen matrices to tinyMatrix (assuming tinyMatrix is Eigen::MatrixXd)
        tinyMatrix A_tiny = A.cast<tinytype>();
        tinyMatrix B_tiny = B.cast<tinytype>();
        tinyMatrix fdyn_tiny = fdyn.cast<tinytype>();
        tinyMatrix Q_tiny = Q.cast<tinytype>();
        tinyMatrix R_tiny = R.cast<tinytype>();
        
        // Setup solver
        int status = tiny_setup(&solver_ptr, A_tiny, B_tiny, fdyn_tiny, Q_tiny, R_tiny, 
                               (tinytype)rho, nx, nu, N, verbose);
        
        if (status != 0) {
            mexErrMsgIdAndTxt("TinyMPC:SetupFailed", "tiny_setup failed with status %d", status);
        }
        
        // Set bounds
        tinyMatrix x_min_tiny = x_min.cast<tinytype>();
        tinyMatrix x_max_tiny = x_max.cast<tinytype>();
        tinyMatrix u_min_tiny = u_min.cast<tinytype>();
        tinyMatrix u_max_tiny = u_max.cast<tinytype>();
        
        status = tiny_set_bound_constraints(solver_ptr, x_min_tiny, x_max_tiny, u_min_tiny, u_max_tiny);
        
        if (status != 0) {
            mexErrMsgIdAndTxt("TinyMPC:BoundsFailed", "tiny_set_bound_constraints failed with status %d", status);
        }
        
        // Store solver (transfer ownership)
        g_solver.reset(solver_ptr);
        
        if (verbose) {
            mexPrintf("TinyMPC solver setup successful\n");
        }
        
        // Return status
        plhs[0] = mxCreateDoubleScalar(0);
        
    } catch (const std::exception& e) {
        mexErrMsgIdAndTxt("TinyMPC:SetupError", "Setup failed: %s", e.what());
    }
}

// Set initial state
void set_x0(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 3) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "set_x0 requires 2 input arguments");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized. Call setup first.");
    }
    
    auto x0 = matlab_to_eigen(prhs[1]);
    int verbose = (int)mxGetScalar(prhs[2]);
    
    try {
        tinyVector x0_tiny = x0.cast<tinytype>();
        int status = tiny_set_x0(g_solver.get(), x0_tiny);
        
        if (status != 0) {
            mexErrMsgIdAndTxt("TinyMPC:SetX0Failed", "tiny_set_x0 failed with status %d", status);
        }
        
        if (verbose) {
            mexPrintf("Initial state set successfully\n");
        }
        
    } catch (const std::exception& e) {
        mexErrMsgIdAndTxt("TinyMPC:SetX0Error", "set_x0 failed: %s", e.what());
    }
}

// Solve the problem
void solve_problem(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "solve requires 1 input argument");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized. Call setup first.");
    }
    
    int verbose = (int)mxGetScalar(prhs[1]);
    
    try {
        if (verbose) {
            mexPrintf("Solving MPC problem...\n");
        }
        
        int status = tiny_solve(g_solver.get());
        
        if (verbose) {
            mexPrintf("Solver finished with status %d\n", status);
        }
        
    } catch (const std::exception& e) {
        mexErrMsgIdAndTxt("TinyMPC:SolveError", "solve failed: %s", e.what());
    }
}

// Get solution
void get_solution(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "get_solution requires no input arguments");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized. Call setup first.");
    }
    
    if (!g_solver->solution) {
        mexErrMsgIdAndTxt("TinyMPC:NoSolution", "No solution available. Call solve first.");
    }
    
    try {
        // Return x_sol, u_sol, solved, iter
        if (nlhs >= 1) {
            // Convert solution matrices back to double and return
            Eigen::MatrixXd x_sol = g_solver->solution->x.cast<double>();
            plhs[0] = eigen_to_matlab(x_sol);
        }
        
        if (nlhs >= 2) {
            Eigen::MatrixXd u_sol = g_solver->solution->u.cast<double>();
            plhs[1] = eigen_to_matlab(u_sol);
        }
        
        if (nlhs >= 3) {
            plhs[2] = mxCreateLogicalScalar(g_solver->solution->solved);
        }
        
        if (nlhs >= 4) {
            plhs[3] = mxCreateDoubleScalar(g_solver->solution->iter);
        }
        
    } catch (const std::exception& e) {
        mexErrMsgIdAndTxt("TinyMPC:GetSolutionError", "get_solution failed: %s", e.what());
    }
}

// Cleanup
void cleanup_solver(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (g_solver) {
        g_solver.reset();
        mexPrintf("TinyMPC solver cleaned up\n");
    }
}

// Main MEX function
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "At least one input argument required");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "First argument must be a string");
    }
    
    // Get command string
    char* command = mxArrayToString(prhs[0]);
    
    if (strcmp(command, "setup") == 0) {
        setup_solver(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(command, "set_x0") == 0) {
        set_x0(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(command, "solve") == 0) {
        solve_problem(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(command, "get_solution") == 0) {
        get_solution(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(command, "cleanup") == 0) {
        cleanup_solver(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgIdAndTxt("TinyMPC:InvalidCommand", "Unknown command: %s", command);
    }
    
    mxFree(command);
}
