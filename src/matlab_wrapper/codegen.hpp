typedef double tinytype;

#ifdef __cplusplus
extern "C" {
#endif

// Main solver setup
int tiny_setup_matlab(void** solver_ptr,
                      tinytype *Adyn, tinytype *Bdyn, tinytype *fdyn,
                      tinytype *Q, tinytype *R, 
                      tinytype rho, int nx, int nu, int N, int verbose);

int tiny_set_bound_constraints_matlab(void* solver,
                                      tinytype *x_min, tinytype *x_max,
                                      tinytype *u_min, tinytype *u_max);

// Code generation functions
int tiny_codegen_matlab(void* solver, const char* output_dir, int verbose);

int tiny_codegen_with_sensitivity_matlab(void* solver, const char* output_dir,
                                         tinytype *dK, tinytype *dP,
                                         tinytype *dC1, tinytype *dC2, int verbose);

#ifdef __cplusplus
}
#endif
