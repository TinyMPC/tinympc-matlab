#include "mex.h" // MATLAB API header
#include <Eigen/Dense>
#include <tinympc/tiny_data_workspace.hpp>
#include <tinympc/admm.hpp>
#include <tinympc/tiny_wrapper.hpp>
#include <iostream>

using namespace Eigen;
// Define your external solver instance
extern TinySolver tiny_data_solver;
extern void set_x(float *x0);
extern void set_xref(float *xref);
extern void reset_dual_variables();
extern void get_x(float *x_soln);
extern void get_u(float *u_soln);
extern void get_Adyn(float *Adyn);
extern void get_Bdyn(float *Bdyn);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs < 1 || nlhs < 0)
    {
        mexErrMsgIdAndTxt("MATLAB:tinympc_wrapper_mex:invalidNumInputsOutputs", "Invalid number of input or output arguments.");
    }

    // Check if the first input is a string
    if (!mxIsChar(prhs[0]))
    {
        mexErrMsgIdAndTxt("MATLAB:tinympc_wrapper_mex:invalidInputType", "Input must be a string.");
    }

    // Get the command string
    char *cmd = mxArrayToString(prhs[0]);

    // Implement your commands
    if (strcmp(cmd, "set_x") == 0)
    {
        // Check the number of input arguments
        if (nrhs != 2)
        {
            mexErrMsgIdAndTxt("MATLAB:tinympc_wrapper_mex:invalidNumInputs", "Invalid number of input arguments for set_x.");
        }

        // Extract data from the second input argument
        float *x0 = (float *)mxGetPr(prhs[1]);

        // Call your set_x function
        set_x(x0);
    }
    else if (strcmp(cmd, "set_xref") == 0)
    {
        // Check the number of input arguments
        if (nrhs != 2)
        {
            mexErrMsgIdAndTxt("MATLAB:tinympc_wrapper_mex:invalidNumInputs", "Invalid number of input arguments for set_xref.");
        }

        // Extract data from the second input argument
        float *xref = (float *)mxGetPr(prhs[1]);

        // Call your set_xref function
        set_xref(xref);
    }
    else if (strcmp(cmd, "reset_dual_variables") == 0)
    {
        // Check the number of input arguments
        if (nrhs != 1)
        {
            mexErrMsgIdAndTxt("MATLAB:tinympc_wrapper_mex:invalidNumInputs", "Invalid number of input arguments for reset_dual_variables.");
        }

        // Call your reset_dual_variables function
        reset_dual_variables();
    } else if (strcmp(cmd, "call_tiny_solve") == 0) {
        // Check the number of input arguments
        if (nrhs != 1) {
            mexErrMsgIdAndTxt("MATLAB:tinympc_wrapper_mex:invalidNumInputs", "Invalid number of input arguments for call_tiny_solve.");
        }

        // Call your call_tiny_solve function
        call_tiny_solve();
    } else if (strcmp(cmd, "get_x") == 0)
    {
        float *x_soln = (float *)mxGetData(prhs[1]);
        get_x(x_soln);
    }
    else if (strcmp(cmd, "get_u") == 0)
    {
        float *u_soln = (float *)mxGetData(prhs[1]);
        get_u(u_soln);
    }
    else if (strcmp(cmd, "get_Adyn") == 0)
    {
        float *Adyn = (float *)mxGetData(prhs[1]);
        get_Adyn(Adyn);
    }
    else if (strcmp(cmd, "get_Bdyn") == 0)
    {
        float *Bdyn = (float *)mxGetData(prhs[1]);
        get_Bdyn(Bdyn);
    }
    // Free allocated memory for the command string
    mxFree(cmd);

    return;
}
