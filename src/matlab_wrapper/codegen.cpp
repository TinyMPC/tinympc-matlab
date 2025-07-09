#include "codegen.hpp"
#include "../tinympc/TinyMPC/src/tinympc/codegen.hpp"
#include "../tinympc/TinyMPC/src/tinympc/tiny_api.hpp"
#include "../tinympc/TinyMPC/src/tinympc/types.hpp"
#include <iostream>

#ifdef __cplusplus
extern "C" {
#endif

int tiny_setup_matlab(void** solver_ptr,
                      tinytype *Adyn, tinytype *Bdyn, tinytype *fdyn,
                      tinytype *Q, tinytype *R, 
                      tinytype rho, int nx, int nu, int N, int verbose) {
    
    if (solver_ptr == nullptr) {
        if (verbose) {
            std::cout << "Error: solver_ptr is null" << std::endl;
        }
        return -1;
    }

    // Allocate and initialize solver
    TinySolver* solver = new TinySolver();
    
    // Convert MATLAB arrays to Eigen matrices
    // MATLAB stores matrices in column-major order, same as Eigen default
    Eigen::Map<tinyMatrix> A_mat(Adyn, nx, nx);
    Eigen::Map<tinyMatrix> B_mat(Bdyn, nx, nu);
    Eigen::Map<tinyVector> f_vec(fdyn, nx);
    Eigen::Map<tinyMatrix> Q_mat(Q, nx, nx);
    Eigen::Map<tinyMatrix> R_mat(R, nu, nu);
    
    // Call TinyMPC core setup function
    int status = tiny_setup(solver, A_mat, B_mat, f_vec, Q_mat, R_mat, rho, N, verbose);
    
    if (status == 0) {
        *solver_ptr = static_cast<void*>(solver);
        if (verbose) {
            std::cout << "Successfully set up TinyMPC solver via MATLAB interface" << std::endl;
            std::cout << "Problem dimensions: nx=" << nx << ", nu=" << nu << ", N=" << N << std::endl;
        }
    } else {
        delete solver;
        *solver_ptr = nullptr;
        if (verbose) {
            std::cout << "Failed to set up TinyMPC solver: " << status << std::endl;
        }
    }
    
    return status;
}

int tiny_set_bound_constraints_matlab(void* solver,
                                      tinytype *x_min, tinytype *x_max,
                                      tinytype *u_min, tinytype *u_max) {
    
    if (solver == nullptr) {
        std::cout << "Error: solver is null" << std::endl;
        return -1;
    }
    
    TinySolver* tiny_solver = static_cast<TinySolver*>(solver);
    
    if (tiny_solver->work == nullptr) {
        std::cout << "Error: solver workspace not initialized" << std::endl;
        return -1;
    }
    
    int nx = tiny_solver->work->nx;
    int nu = tiny_solver->work->nu;
    int N = tiny_solver->work->N;
    
    // Convert MATLAB arrays to Eigen matrices
    Eigen::Map<tinyMatrix> x_min_mat(x_min, nx, N);
    Eigen::Map<tinyMatrix> x_max_mat(x_max, nx, N);
    Eigen::Map<tinyMatrix> u_min_mat(u_min, nu, N-1);
    Eigen::Map<tinyMatrix> u_max_mat(u_max, nu, N-1);
    
    // Call TinyMPC core function
    return tiny_set_bound_constraints(tiny_solver, x_min_mat, x_max_mat, u_min_mat, u_max_mat);
}

int tiny_codegen_matlab(void* solver, const char* output_dir, int verbose) {
    if (solver == nullptr) {
        if (verbose) {
            std::cout << "Error: solver is null" << std::endl;
        }
        return -1;
    }
    
    TinySolver* tiny_solver = static_cast<TinySolver*>(solver);
    
    if (verbose) {
        std::cout << "Starting MATLAB code generation to directory: " << output_dir << std::endl;
    }
    
    // Call TinyMPC core codegen function
    int status = tiny_codegen(tiny_solver, output_dir, verbose);
    
    if (verbose) {
        if (status == 0) {
            std::cout << "Code generation completed successfully" << std::endl;
        } else {
            std::cout << "Code generation failed with status: " << status << std::endl;
        }
    }
    
    return status;
}

int tiny_codegen_with_sensitivity_matlab(void* solver, const char* output_dir,
                                         tinytype *dK, tinytype *dP,
                                         tinytype *dC1, tinytype *dC2, int verbose) {
    
    if (solver == nullptr) {
        if (verbose) {
            std::cout << "Error: solver is null" << std::endl;
        }
        return -1;
    }
    
    TinySolver* tiny_solver = static_cast<TinySolver*>(solver);
    
    if (tiny_solver->work == nullptr) {
        if (verbose) {
            std::cout << "Error: solver workspace not initialized" << std::endl;
        }
        return -1;
    }
    
    int nx = tiny_solver->work->nx;
    int nu = tiny_solver->work->nu;
    int N = tiny_solver->work->N;
    
    if (verbose) {
        std::cout << "Starting MATLAB code generation with sensitivity matrices to directory: " << output_dir << std::endl;
    }
    
    // Convert MATLAB arrays to Eigen matrices
    // These dimensions should match what's expected by the core function
    Eigen::Map<tinyMatrix> dK_mat(dK, nu * nx, N-1);  // Control gain sensitivity
    Eigen::Map<tinyMatrix> dP_mat(dP, nx * nx, N);    // Cost-to-go sensitivity
    Eigen::Map<tinyMatrix> dC1_mat(dC1, nu, nx);      // Constraint sensitivity 1
    Eigen::Map<tinyMatrix> dC2_mat(dC2, nx, nx);      // Constraint sensitivity 2
    
    // Call TinyMPC core codegen function with sensitivity
    int status = tiny_codegen_with_sensitivity(tiny_solver, output_dir, 
                                               &dK_mat, &dP_mat, &dC1_mat, &dC2_mat, verbose);
    
    if (verbose) {
        if (status == 0) {
            std::cout << "Code generation with sensitivity completed successfully" << std::endl;
        } else {
            std::cout << "Code generation with sensitivity failed with status: " << status << std::endl;
        }
    }
    
    return status;
}

#ifdef __cplusplus
}
#endif
