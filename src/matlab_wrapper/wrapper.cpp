#include "wrapper.hpp"
#include "../tinympc/TinyMPC/src/tinympc/tiny_api.hpp"
#include "../tinympc/TinyMPC/src/tinympc/types.hpp"
#include <iostream>

// Global solver pointer, probably a better way to do this
static TinySolver* g_solver = nullptr;

// EXAMPLE IMPLEMENTATION: set_x0
void set_x0(double *x0, int verbose) {
    if (g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }
    
    if (x0 == nullptr) {
        if (verbose) {
            std::cout << "Error: x0 array is null." << std::endl;
        }
        return;
    }
    
    // Convert double array to Eigen vector
    int nx = g_solver->work->nx;
    
    // Create Eigen map from the double array
    Eigen::Map<tinyVector> x0_vec(x0, nx);
    
    // Call the TinyMPC core function
    int status = tiny_set_x0(g_solver, x0_vec);
    
    if (verbose) {
        if (status == 0) {
            std::cout << "Successfully set initial state x0" << std::endl;
        } else {
            std::cout << "Error setting initial state: " << status << std::endl;
        }
    }
}

void call_tiny_solve(int verbose) {
    if(g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }
    int status = tiny_solve(g_solver);
    if (verbose) {
        if (status == 0) {
            std::cout << "Successfully called tiny_solve" << std::endl;
        } else {
            std::cout << "Error calling tiny_solve: " << status << std::endl;
        }
    }
}

void get_x(double *x_soln, int verbose) {
    if (g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }
    
    if (x_soln == nullptr) {
        if (verbose) {
            std::cout << "Error: x_soln array is null." << std::endl;
        }
        return;
    }
    
    if (g_solver->solution == nullptr) {
        if (verbose) {
            std::cout << "Error: No solution available. Call solve first." << std::endl;
        }
        return;
    }
    
    // Get problem dimensions
    int nx = g_solver->work->nx;
    int N = g_solver->work->N;
    
    // Create Eigen map for the output array (MATLAB will allocate this)
    Eigen::Map<tinyMatrix> x_soln_mat(x_soln, nx, N);
    
    // Copy solution matrix to output
    x_soln_mat = g_solver->solution->x;
    
    if (verbose) {
        std::cout << "Successfully retrieved state trajectory" << std::endl;
        std::cout << "State matrix size: " << nx << " x " << N << std::endl;
        std::cout << "First state: [" << g_solver->solution->x.col(0).transpose() << "]" << std::endl;
    }
}

void get_u(double *u_soln, int verbose) {
    if (g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }
    
    if (u_soln == nullptr) {
        if (verbose) {
            std::cout << "Error: u_soln array is null." << std::endl;
        }
        return;
    }
    
    if (g_solver->solution == nullptr) {
        if (verbose) {
            std::cout << "Error: No solution available. Call solve first." << std::endl;
        }
        return;
    }
    
    // Get problem dimensions
    int nu = g_solver->work->nu;
    int N = g_solver->work->N;
    
    // Create Eigen map for the output array (MATLAB will allocate this)
    Eigen::Map<tinyMatrix> u_soln_mat(u_soln, nu, N - 1);
    
    // Copy solution matrix to output
    u_soln_mat = g_solver->solution->u;
    
    if (verbose) {
        std::cout << "Successfully retrieved control trajectory" << std::endl;
        std::cout << "Control matrix size: " << nu << " x " << (N - 1) << std::endl;
        std::cout << "First control: [" << g_solver->solution->u.col(0).transpose() << "]" << std::endl;
    }
}

void set_x_ref(double *x_ref, int verbose) {
    if (g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }
    
    if (x_ref == nullptr) {
        if (verbose) {
            std::cout << "Error: x_ref array is null." << std::endl;
        }
        return;
    }
    
    // Get problem dimensions
    int nx = g_solver->work->nx;
    int N = g_solver->work->N;
    
    // Convert double array to Eigen matrix (nx x N)
    // MATLAB stores matrices in column-major order, same as Eigen
    Eigen::Map<tinyMatrix> x_ref_mat(x_ref, nx, N);
    
    // Call the TinyMPC core function
    int status = tiny_set_x_ref(g_solver, x_ref_mat);
    
    if (verbose) {
        if (status == 0) {
            std::cout << "Successfully set state reference trajectory" << std::endl;
            std::cout << "Reference matrix size: " << nx << " x " << N << std::endl;
        } else {
            std::cout << "Error setting state reference: " << status << std::endl;
        }
    }
}

void set_u_ref(double *u_ref, int verbose) {
    if (g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }

    if (u_ref == nullptr) {
        if (verbose) {
            std::cout << "Error: u_ref array is null." << std::endl;
        }
        return;
    }

    // Get problem dimensions
    int nu = g_solver->work->nu;
    int N = g_solver->work->N;

    // Convert double array to Eigen matrix (nu x N-1)
    // MATLAB stores matrices in column-major order, same as Eigen
    Eigen::Map<tinyMatrix> u_ref_mat(u_ref, nu, N-1);

    // Call the TinyMPC core function
    int status = tiny_set_u_ref(g_solver, u_ref_mat);
    
    if (verbose) {
        if (status == 0) {
            std::cout << "Successfully set control reference trajectory" << std::endl;
            std::cout << "Reference matrix size: " << nu << " x " << N-1 << std::endl;
        } else {
            std::cout << "Error setting control reference: " << status << std::endl;
        }
    }
}

void set_bound_constraints(double *x_min, double *x_max, 
                          double *u_min, double *u_max, int verbose) {
    if (g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }

    if (x_min == nullptr || x_max == nullptr || u_min == nullptr || u_max == nullptr) {
        if (verbose) {
            std::cout << "Error: One or more constraint arrays are null." << std::endl;
        }
        return;
    }

    int nx = g_solver->work->nx;  // State dimension    
    int nu = g_solver->work->nu;  // Control dimension
    int N = g_solver->work->N;    // Horizon length

    // Convert arrays to Eigen matrices
    Eigen::Map<tinyMatrix> x_min_mat(x_min, nx, N);
    Eigen::Map<tinyMatrix> x_max_mat(x_max, nx, N);
    Eigen::Map<tinyMatrix> u_min_mat(u_min, nu, N-1);
    Eigen::Map<tinyMatrix> u_max_mat(u_max, nu, N-1);

    // Call the TinyMPC core function
    int status = tiny_set_bound_constraints(g_solver, x_min_mat, x_max_mat, u_min_mat, u_max_mat);

    if (verbose) {
        if (status == 0) {
            std::cout << "Successfully set bound constraints" << std::endl;
        } else {
            std::cout << "Error setting bound constraints: " << status << std::endl;
        }
    }
}

void set_sensitivity_matrices(double *dK, double *dP, 
                             double *dC1, double *dC2, int verbose) {
    if (g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }
    
    if (g_solver->cache == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver cache not initialized." << std::endl;
        }
        return;
    }
    
    // Get problem dimensions
    int nx = g_solver->work->nx;
    int nu = g_solver->work->nu;
    int N = g_solver->work->N;
    
    // Convert arrays to Eigen matrices and store in cache
    // Based on Python bindings approach: create copies and store for code generation
    
    if (dK != nullptr) {
        // dK: Control gain sensitivity (nu x nx for each time step)
        // Assuming flattened format: nu*nx rows, N-1 columns
        Eigen::Map<tinyMatrix> dK_mat(dK, nu * nx, N-1);
        g_solver->cache->dKinf_drho = dK_mat;
        if (verbose) {
            std::cout << "Set dK sensitivity matrix (" << nu*nx << " x " << N-1 << ")" << std::endl;
            std::cout << "dK norm: " << dK_mat.norm() << std::endl;
        }
    }
    
    if (dP != nullptr) {
        // dP: Cost-to-go sensitivity (nx x nx for each time step)
        // Assuming flattened format: nx*nx rows, N columns  
        Eigen::Map<tinyMatrix> dP_mat(dP, nx * nx, N);
        g_solver->cache->dPinf_drho = dP_mat;
        if (verbose) {
            std::cout << "Set dP sensitivity matrix (" << nx*nx << " x " << N << ")" << std::endl;
            std::cout << "dP norm: " << dP_mat.norm() << std::endl;
        }
    }
    
    if (dC1 != nullptr) {
        // dC1: Constraint sensitivity matrix 1
        // Dimensions depend on constraint structure - use cache C1 size as reference
        Eigen::Map<tinyMatrix> dC1_mat(dC1, nu, nx);  // Assuming nu x nx like Kinf
        g_solver->cache->dC1_drho = dC1_mat;
        if (verbose) {
            std::cout << "Set dC1 sensitivity matrix (" << nu << " x " << nx << ")" << std::endl;
            std::cout << "dC1 norm: " << dC1_mat.norm() << std::endl;
        }
    }
    
    if (dC2 != nullptr) {
        // dC2: Constraint sensitivity matrix 2
        // Dimensions depend on constraint structure - use cache C2 size as reference
        Eigen::Map<tinyMatrix> dC2_mat(dC2, nx, nx);  // Assuming nx x nx like AmBKt
        g_solver->cache->dC2_drho = dC2_mat;
        if (verbose) {
            std::cout << "Set dC2 sensitivity matrix (" << nx << " x " << nx << ")" << std::endl;
            std::cout << "dC2 norm: " << dC2_mat.norm() << std::endl;
        }
    }
    
    if (verbose) {
        std::cout << "Sensitivity matrices set for adaptive rho and code generation" << std::endl;
        std::cout << "These matrices will be used for:" << std::endl;
        std::cout << "  - Adaptive rho updates during solve" << std::endl;
        std::cout << "  - Code generation with sensitivity information" << std::endl;
    }
}

void set_cache_terms(double *Kinf, double *Pinf, 
                    double *Quu_inv, double *AmBKt, int verbose) {
    if (g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }
    
    if (g_solver->cache == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver cache not initialized." << std::endl;
        }
        return;
    }
    
    // Get problem dimensions
    int nx = g_solver->work->nx;
    int nu = g_solver->work->nu;
    
    // Convert arrays to Eigen matrices and set cache terms
    
    // Kinf: Infinite horizon feedback gain (nu x nx)
    if (Kinf != nullptr) {
        Eigen::Map<tinyMatrix> Kinf_mat(Kinf, nu, nx);
        g_solver->cache->Kinf = Kinf_mat;
        if (verbose) {
            std::cout << "Set Kinf (" << nu << " x " << nx << ")" << std::endl;
        }
    }
    
    // Pinf: Infinite horizon cost-to-go (nx x nx)
    if (Pinf != nullptr) {
        Eigen::Map<tinyMatrix> Pinf_mat(Pinf, nx, nx);
        g_solver->cache->Pinf = Pinf_mat;
        if (verbose) {
            std::cout << "Set Pinf (" << nx << " x " << nx << ")" << std::endl;
        }
    }
    
    // Quu_inv: Inverted control cost matrix (nu x nu)
    if (Quu_inv != nullptr) {
        Eigen::Map<tinyMatrix> Quu_inv_mat(Quu_inv, nu, nu);
        g_solver->cache->Quu_inv = Quu_inv_mat;
        if (verbose) {
            std::cout << "Set Quu_inv (" << nu << " x " << nu << ")" << std::endl;
        }
    }
    
    // AmBKt: A - B*K transpose (nx x nx)
    if (AmBKt != nullptr) {
        Eigen::Map<tinyMatrix> AmBKt_mat(AmBKt, nx, nx);
        g_solver->cache->AmBKt = AmBKt_mat;
        if (verbose) {
            std::cout << "Set AmBKt (" << nx << " x " << nx << ")" << std::endl;
        }
    }
    
    if (verbose) {
        std::cout << "Successfully updated solver cache terms" << std::endl;
    }
}

void reset_dual_variables(int verbose) {
    if (g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return;
    }
    
    if (g_solver->work == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver workspace not initialized." << std::endl;
        }
        return;
    }
    
    // Reset all dual variables to zero (manual implementation since no core function exists)
    // Based on TinyWorkspace structure in types.hpp
    
    // Bound constraint dual variables
    g_solver->work->g.setZero();  // State dual variables (nx x N)
    g_solver->work->y.setZero();  // Input dual variables (nu x N-1)
    
    // Cone constraint dual variables (if applicable)
    if (g_solver->work->numStateCones > 0 || g_solver->work->numInputCones > 0) {
        g_solver->work->gc.setZero();  // State cone dual variables (nx x N)
        g_solver->work->yc.setZero();  // Input cone dual variables (nu x N-1)
    }
    
    // Linear constraint dual variables (if applicable)
    if (g_solver->work->numStateLinear > 0 || g_solver->work->numInputLinear > 0) {
        g_solver->work->gl.setZero();  // State linear dual variables (nx x N)
        g_solver->work->yl.setZero();  // Input linear dual variables (nu x N-1)
    }
    
    // Reset solve status
    g_solver->work->iter = 0;
    g_solver->work->status = 0;
    
    if (verbose) {
        std::cout << "Successfully reset all ADMM dual variables to zero" << std::endl;
        std::cout << "Reset g (state duals): " << g_solver->work->nx << " x " << g_solver->work->N << std::endl;
        std::cout << "Reset y (input duals): " << g_solver->work->nu << " x " << (g_solver->work->N - 1) << std::endl;
        if (g_solver->work->numStateCones > 0 || g_solver->work->numInputCones > 0) {
            std::cout << "Reset cone constraint dual variables" << std::endl;
        }
        if (g_solver->work->numStateLinear > 0 || g_solver->work->numInputLinear > 0) {
            std::cout << "Reset linear constraint dual variables" << std::endl;
        }
    }
}

int get_solver_status(int verbose) {
    if(g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return -1;
    }

    if(g_solver->solution == nullptr) {
        if (verbose) {
            std::cout << "Error: No solution available. Call solve first." << std::endl;
        }
        return -1;
    }

    return g_solver->solution->solved;
}

int get_solver_iterations(int verbose) {
    if(g_solver == nullptr) {
        if (verbose) {
            std::cout << "Error: Solver not initialized. Call setup first." << std::endl;
        }
        return -1;
    }
    if(g_solver->solution == nullptr) {
        if (verbose) {
            std::cout << "Error: No solution available. Call solve first." << std::endl;
        }
        return -1;
    }
    return g_solver->solution->iter;
}
