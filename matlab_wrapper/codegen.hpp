typedef double tinytype;

int tiny_codegen(int nx, int nu, int N,
                     tinytype *Adyn, tinytype *Bdyn, tinytype *Q, tinytype *R,
                     tinytype *x_min, tinytype *x_max, tinytype *u_min, tinytype *u_max,
                     tinytype rho, tinytype abs_pri_tol, tinytype abs_dua_tol, 
                     int max_iters, int check_termination, int gen_wrapper,
                     const char *tinympc_dir, const char *output_dir);
