#include "mex.h"
#include <Eigen/Dense>
#include "src/tinympc/codegen.hpp"
#include <stdio.h>
#include <iostream>
using namespace Eigen;

typedef double tinytype;

int tiny_codegen(int nx, int nu, int N,
                 tinytype *Adyn, tinytype *Bdyn, tinytype *Q, tinytype *Qf, tinytype *R,
                 tinytype *x_min, tinytype *x_max, tinytype *u_min, tinytype *u_max,
                 tinytype rho, tinytype abs_pri_tol, tinytype abs_dua_tol, int max_iters, int check_termination,
                 const char* tinympc_dir, const char* output_dir);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Check for the correct number of input and output arguments
    if (nrhs != 19 || nlhs != 1)
    {
        mexErrMsgIdAndTxt("MEXFunction:InvalidInputOutput", "Incorrect number of input or output arguments.");
        return;
    }

    // Extract input data from MATLAB mxArray objects
    int nx = static_cast<int>(mxGetScalar(prhs[0]));
    int nu = static_cast<int>(mxGetScalar(prhs[1]));
    int N = static_cast<int>(mxGetScalar(prhs[2]));

    tinytype *Adyn = static_cast<tinytype*>(mxGetData(prhs[3]));
    tinytype *Bdyn = static_cast<tinytype*>(mxGetData(prhs[4]));
    tinytype *Q = static_cast<tinytype*>(mxGetData(prhs[5]));
    tinytype *Qf = static_cast<tinytype*>(mxGetData(prhs[6]));
    tinytype *R = static_cast<tinytype*>(mxGetData(prhs[7]));
    tinytype *x_min = static_cast<tinytype*>(mxGetData(prhs[8]));
    tinytype *x_max = static_cast<tinytype*>(mxGetData(prhs[9]));
    tinytype *u_min = static_cast<tinytype*>(mxGetData(prhs[10]));
    tinytype *u_max = static_cast<tinytype*>(mxGetData(prhs[11]));
    tinytype rho = static_cast<tinytype>(mxGetScalar(prhs[12]));
    tinytype abs_pri_tol = static_cast<tinytype>(mxGetScalar(prhs[13]));
    tinytype abs_dua_tol = static_cast<tinytype>(mxGetScalar(prhs[14]));
    int max_iters = static_cast<int>(mxGetScalar(prhs[15]));
    int check_termination = static_cast<int>(mxGetScalar(prhs[16]));
    char* tinympc_dir = mxArrayToString(prhs[17]);
    char* output_dir = mxArrayToString(prhs[18]);
    printf("tinympc_dir file path:\n\t%s\n",  tinympc_dir);
    printf("output_dir file path:\n\t%s\n",  output_dir);
    //std::cout<<tinympc_dir;
    //std::cout<<output_dir;

    // Call the tiny_codegen function
    int result = tiny_codegen(nx, nu, N, Adyn, Bdyn, Q, Qf, R, x_min, x_max, u_min, u_max, rho, abs_pri_tol, abs_dua_tol, max_iters, check_termination, tinympc_dir, output_dir);

    // Create the output mxArray for the result
    plhs[0] = mxCreateDoubleScalar(static_cast<double>(result));

    // Clean up memory for char arrays
    mxFree(tinympc_dir);
    mxFree(output_dir);
}
