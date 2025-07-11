#include "mex.h"
#include "matrix.h"
#include <iostream>
#include <cstring>
#include <memory>
#include <filesystem>

// Fix the Eigen include issue by using the standard approach
#include <Eigen/Dense>
#include <Eigen/Core>

// Now include TinyMPC headers (after Eigen is properly included)
#include "tinympc/tiny_api.hpp"
#include "tinympc/types.hpp"
#include "tinympc/codegen.hpp"

// Global solver pointer - matches Python structure
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

// Setup function - initialize the solver (matches Python setup)
void setup_solver(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected arguments: A, B, fdyn, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, verbose
    if (nrhs != 14) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "Setup requires 14 input arguments");
    }
    
    // Extract problem dimensions
    int nx = (int)mxGetScalar(prhs[6]);
    int nu = (int)mxGetScalar(prhs[7]);
    int N = (int)mxGetScalar(prhs[8]);
    
    // Extract matrices
    auto A = matlab_to_eigen(prhs[0]);
    auto B = matlab_to_eigen(prhs[1]);
    auto fdyn = matlab_to_eigen(prhs[2]);
    auto Q = matlab_to_eigen(prhs[3]);
    auto R = matlab_to_eigen(prhs[4]);
    double rho = mxGetScalar(prhs[5]);
    
    // Extract bound matrices
    auto x_min = matlab_to_eigen(prhs[9]);
    auto x_max = matlab_to_eigen(prhs[10]);
    auto u_min = matlab_to_eigen(prhs[11]);
    auto u_max = matlab_to_eigen(prhs[12]);
    
    int verbose = (int)mxGetScalar(prhs[13]);
    
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
        mexErrMsgIdAndTxt("TinyMPC:SetupException", "Setup failed: %s", e.what());
    }
}

// Set initial state
void set_x0(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "set_x0 requires 2 input arguments");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized");
    }
    
    auto x0 = matlab_to_eigen(prhs[0]);
    int verbose = (int)mxGetScalar(prhs[1]);
    
    tinyVector x0_tiny = x0.cast<tinytype>();
    
    // Use the API function
    int status = tiny_set_x0(g_solver.get(), x0_tiny);
    
    if (status != 0) {
        mexErrMsgIdAndTxt("TinyMPC:SetX0Failed", "tiny_set_x0 failed with status %d", status);
    }
    
    if (verbose) {
        mexPrintf("Initial state set\n");
    }
}

// Set state reference
void set_x_ref(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "set_x_ref requires 2 input arguments");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized");
    }
    
    auto x_ref = matlab_to_eigen(prhs[0]);
    int verbose = (int)mxGetScalar(prhs[1]);
    
    tinyMatrix x_ref_tiny = x_ref.cast<tinytype>();
    
    // Use the API function
    int status = tiny_set_x_ref(g_solver.get(), x_ref_tiny);
    
    if (status != 0) {
        mexErrMsgIdAndTxt("TinyMPC:SetXRefFailed", "tiny_set_x_ref failed with status %d", status);
    }
    
    if (verbose) {
        mexPrintf("State reference set\n");
    }
}

// Set input reference
void set_u_ref(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "set_u_ref requires 2 input arguments");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized");
    }
    
    auto u_ref = matlab_to_eigen(prhs[0]);
    int verbose = (int)mxGetScalar(prhs[1]);
    
    tinyMatrix u_ref_tiny = u_ref.cast<tinytype>();
    
    // Use the API function
    int status = tiny_set_u_ref(g_solver.get(), u_ref_tiny);
    
    if (status != 0) {
        mexErrMsgIdAndTxt("TinyMPC:SetURefFailed", "tiny_set_u_ref failed with status %d", status);
    }
    
    if (verbose) {
        mexPrintf("Input reference set\n");
    }
}

// Solve the MPC problem
void solve_mpc(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "solve requires 1 input argument");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized");
    }
    
    int verbose = (int)mxGetScalar(prhs[0]);
    
    // Solve the problem
    int status = tiny_solve(g_solver.get());
    
    if (verbose) {
        mexPrintf("Solve completed with status: %d\n", status);
    }
    
    // Return status
    plhs[0] = mxCreateDoubleScalar(status);
}

// Get solution
void get_solution(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "get_solution requires 1 input argument");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized");
    }
    
    int verbose = (int)mxGetScalar(prhs[0]);
    
    // Get solution from solver
    tinyMatrix x_sol = g_solver->solution->x;
    tinyMatrix u_sol = g_solver->solution->u;
    
    // Convert to double precision for MATLAB
    Eigen::MatrixXd x_sol_double = x_sol.cast<double>();
    Eigen::MatrixXd u_sol_double = u_sol.cast<double>();
    
    // Return solution
    plhs[0] = eigen_to_matlab(x_sol_double);
    plhs[1] = eigen_to_matlab(u_sol_double);
    
    if (verbose) {
        mexPrintf("Solution retrieved\n");
    }
}

// Get solver statistics
void get_stats(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "get_stats requires 1 input argument");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized");
    }
    
    int verbose = (int)mxGetScalar(prhs[0]);
    
    // Get solver statistics
    plhs[0] = mxCreateDoubleScalar(g_solver->work->iter);
    plhs[1] = mxCreateDoubleScalar(g_solver->work->status);
    plhs[2] = mxCreateDoubleScalar(g_solver->work->primal_residual_state);
    plhs[3] = mxCreateDoubleScalar(g_solver->work->primal_residual_input);
    
    if (verbose) {
        mexPrintf("Statistics retrieved: iter=%d, status=%d\n", 
                  g_solver->work->iter, g_solver->work->status);
    }
}

// Code generation
void codegen(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "codegen requires 2 input arguments");
    }
    
    if (!g_solver) {
        mexErrMsgIdAndTxt("TinyMPC:NotInitialized", "Solver not initialized");
    }
    
    // Get output directory
    char* output_dir = mxArrayToString(prhs[0]);
    int verbose = (int)mxGetScalar(prhs[1]);
    
    if (verbose) {
        mexPrintf("Starting code generation to: %s\n", output_dir);
    }
    
    int status = tiny_codegen(g_solver.get(), output_dir, verbose);
    
    if (status != 0) {
        if (verbose) {
            mexPrintf("C++ code generation failed with status: %d\n", status);
        }
        plhs[0] = mxCreateDoubleScalar(status);
        mxFree(output_dir);
        return;
    }
    
    // Copy additional build artifacts (Eigen headers, TinyMPC headers/sources, CMakeLists.txt)
    std::filesystem::path output_path(output_dir);
    
    try {
        // Get the path to the current MEX file to locate the codegen_src directory
        std::string mex_path = __FILE__;
        std::filesystem::path mex_dir = std::filesystem::path(mex_path).parent_path();
        std::filesystem::path codegen_src_path = mex_dir / "codegen_src";
        std::filesystem::path wrapper_path = mex_dir / "wrapper";
        
        if (verbose) {
            mexPrintf("Copying additional build artifacts...\n");
            mexPrintf("MEX directory: %s\n", mex_dir.string().c_str());
            mexPrintf("Codegen source path: %s\n", codegen_src_path.string().c_str());
            mexPrintf("Wrapper path: %s\n", wrapper_path.string().c_str());
        }
        
        // Copy codegen_src contents (include/ and tinympc/ directories)
        if (std::filesystem::exists(codegen_src_path)) {
            std::filesystem::copy(codegen_src_path, output_path, 
                std::filesystem::copy_options::recursive | 
                std::filesystem::copy_options::overwrite_existing);
            
            if (verbose) {
                mexPrintf("Copied codegen_src directory (Eigen headers and TinyMPC sources)\n");
            }
        } else {
            if (verbose) {
                mexPrintf("Warning: codegen_src directory not found at: %s\n", codegen_src_path.string().c_str());
            }
        }
        
        // Copy wrapper files (CMakeLists.txt)
        if (std::filesystem::exists(wrapper_path)) {
            for (const auto& entry : std::filesystem::directory_iterator(wrapper_path)) {
                if (entry.is_regular_file()) {
                    std::filesystem::copy_file(entry.path(), output_path / entry.path().filename(),
                        std::filesystem::copy_options::overwrite_existing);
                    
                    if (verbose) {
                        mexPrintf("Copied wrapper file: %s\n", entry.path().filename().string().c_str());
                    }
                }
            }
        } else {
            if (verbose) {
                mexPrintf("Warning: wrapper directory not found at: %s\n", wrapper_path.string().c_str());
            }
        }
        
        // Create empty build directory for convenience
        std::filesystem::path build_path = output_path / "build";
        if (!std::filesystem::exists(build_path)) {
            std::filesystem::create_directory(build_path);
            if (verbose) {
                mexPrintf("Created empty build directory\n");
            }
        }
        
        if (verbose) {
            mexPrintf("Successfully copied all build artifacts\n");
        }
        
    } catch (const std::filesystem::filesystem_error& e) {
        if (verbose) {
            mexPrintf("Error copying build artifacts: %s\n", e.what());
        }
        // Continue anyway since basic code generation succeeded
    }
    
    // Return status
    plhs[0] = mxCreateDoubleScalar(status);
    
    if (verbose) {
        mexPrintf("Code generation completed with status: %d\n", status);
    }
    
    mxFree(output_dir);
}

// Reset solver
void reset_solver(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 1) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "reset requires 1 input argument");
    }
    
    int verbose = (int)mxGetScalar(prhs[0]);
    
    if (g_solver) {
        g_solver.reset();
        if (verbose) {
            mexPrintf("Solver reset\n");
        }
    }
}

// Main MEX function entry point
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("TinyMPC:InvalidInput", "At least one input argument required");
    }
    
    // Get function name
    char* func_name = mxArrayToString(prhs[0]);
    
    try {
        if (strcmp(func_name, "setup") == 0) {
            setup_solver(nlhs, plhs, nrhs-1, prhs+1);
        } else if (strcmp(func_name, "set_x0") == 0) {
            set_x0(nlhs, plhs, nrhs-1, prhs+1);
        } else if (strcmp(func_name, "set_x_ref") == 0) {
            set_x_ref(nlhs, plhs, nrhs-1, prhs+1);
        } else if (strcmp(func_name, "set_u_ref") == 0) {
            set_u_ref(nlhs, plhs, nrhs-1, prhs+1);
        } else if (strcmp(func_name, "solve") == 0) {
            solve_mpc(nlhs, plhs, nrhs-1, prhs+1);
        } else if (strcmp(func_name, "get_solution") == 0) {
            get_solution(nlhs, plhs, nrhs-1, prhs+1);
        } else if (strcmp(func_name, "get_stats") == 0) {
            get_stats(nlhs, plhs, nrhs-1, prhs+1);
        } else if (strcmp(func_name, "codegen") == 0) {
            codegen(nlhs, plhs, nrhs-1, prhs+1);
        } else if (strcmp(func_name, "reset") == 0) {
            reset_solver(nlhs, plhs, nrhs-1, prhs+1);
        } else {
            mexErrMsgIdAndTxt("TinyMPC:InvalidFunction", "Unknown function: %s", func_name);
        }
    } catch (const std::exception& e) {
        mexErrMsgIdAndTxt("TinyMPC:Exception", "Error: %s", e.what());
    }
    
    mxFree(func_name);
}
