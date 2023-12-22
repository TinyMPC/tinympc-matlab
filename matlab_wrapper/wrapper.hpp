
    void set_x0(float *x0, int verbose);
    void set_xref(float *xref, int verbose);
    void set_umin(float *umin, int verbose);
    void set_umax(float *umax, int verbose);
    void set_xmin(float *xmin, int verbose);
    void set_xmax(float *xmax, int verbose);
    void reset_dual_variables(int verbose);
    void call_tiny_solve(int verbose);
    void get_x(float *x_soln, int verbose);
    void get_u(float *u_soln, int verbose);