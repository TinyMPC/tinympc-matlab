#pragma once

// Include necessary headers
#include <tinympc/tiny_data_workspace.hpp>
#include <tinympc/admm.hpp>

// Declare your functions
extern "C" {
    void set_x(float *x0);
    void set_xref(float *xref);
    void reset_dual_variables();
    void call_tiny_solve();
    void get_x(float *x_soln);
    void get_u(float *u_soln);
    void edit_x(float *x);
    void get_Adyn(float *A_dyn);
    void get_Bdyn(float *B_dyn);
}
