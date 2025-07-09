
#ifdef __cplusplus
extern "C" {
#endif

// Basic runtime operations
void set_x0(double *x0, int verbose);
void call_tiny_solve(int verbose);
void get_x(double *x_soln, int verbose);
void get_u(double *u_soln, int verbose);

// Reference trajectory support (NEW)
void set_x_ref(double *x_ref, int verbose);
void set_u_ref(double *u_ref, int verbose);

// Constraint updates 
void set_bound_constraints(double *x_min, double *x_max, 
                          double *u_min, double *u_max, int verbose);

// Advanced features (NEW)
void set_sensitivity_matrices(double *dK, double *dP, 
                             double *dC1, double *dC2, int verbose);
void set_cache_terms(double *Kinf, double *Pinf, 
                    double *Quu_inv, double *AmBKt, int verbose);

// Solver management
void reset_dual_variables(int verbose);
int get_solver_status(int verbose);
int get_solver_iterations(int verbose);


#ifdef __cplusplus
}
#endif