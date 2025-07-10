% MATLAB equivalent of cartpole_example_code_generation.py
% Code generation example for cartpole stabilization

clear all; close all; clc;

fprintf('=== TinyMPC MATLAB: Cartpole Code Generation Example ===\n\n');

% Same cartpole dynamics as Python example
A = [1.0, 0.01, 0.0, 0.0;
     0.0, 1.0, 0.039, 0.0;
     0.0, 0.0, 1.002, 0.01;
     0.0, 0.0, 0.458, 1.002];

B = [0.0;
     0.02;
     0.0;
     0.067];

Q = diag([10.0, 1, 10, 1]);
R = diag([1.0]);

N = 20;

% Problem dimensions
nx = size(A, 1);
nu = size(B, 2);

% Affine dynamics term (zeros for linear system)
fdyn = zeros(nx, 1);

% Input constraints (matching Python example)
u_min_val = -0.5;
u_max_val = 0.5;

% Set up bounds
infty = 1e17;
x_min = -infty * ones(nx, N);
x_max = infty * ones(nx, N);
u_min = u_min_val * ones(nu, N-1);
u_max = u_max_val * ones(nu, N-1);

% Setup parameters
rho = 1.0;
verbose = 1;  % Set to 1 for verbose output

fprintf('Setting up TinyMPC solver with constraints [%.2f, %.2f]...\n', u_min_val, u_max_val);

% Setup solver
status = tinympc_matlab('setup', A, B, fdyn, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, verbose);

if status ~= 0
    error('Setup failed with status %d', status);
end

% Generate code
fprintf('Generating C code...\n');
codegen_status = tinympc_matlab('codegen', 'out', verbose);

if codegen_status == 0
    fprintf('✅ Code generation completed successfully!\n');
    fprintf('Generated files in "out" directory:\n');
    
    % List generated files if directory exists
    if exist('out', 'dir')
        files = dir('out');
        for i = 1:length(files)
            if ~files(i).isdir
                fprintf('  - %s\n', files(i).name);
            end
        end
    end
else
    fprintf('❌ Code generation failed with status %d\n', codegen_status);
end

% Clean up
tinympc_matlab('reset', verbose);

fprintf('\nCode generation example completed!\n');
