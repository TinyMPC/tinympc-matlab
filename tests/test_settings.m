% Test solver setup with custom options
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'matlab_wrapper'));

A = [1.0, 0.01, 0.0, 0.0;
     0.0, 1.0, 0.039, 0.0;
     0.0, 0.0, 1.002, 0.01;
     0.0, 0.0, 0.458, 1.002];
B = [0.0; 0.02; 0.0; 0.067];
Q = diag([10.0, 1, 10, 1]);
R = diag([1.0]);
N = 2;

try
    prob = TinyMPC();
    prob.setup(A, B, Q, R, N, 'rho', 1.5, 'max_iter', 50);
    fprintf('test_settings.m: PASSED\n');
catch ME
    if contains(ME.message, 'tinympc_matlab')
        fprintf('test_settings.m: SKIPPED (MEX file not compiled)\n');
    else
        fprintf('test_settings.m: ERROR - %s\n', ME.message);
        rethrow(ME);
    end
end
