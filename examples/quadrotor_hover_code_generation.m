% MATLAB equivalent of quadrotor_hover_code_generation.py
% Code generation example for quadrotor hover control

clear all; close all; clc;

fprintf('=== TinyMPC MATLAB: Quadrotor Hover Code Generation Example ===\n\n');

% Toggle switch for adaptive rho
ENABLE_ADAPTIVE_RHO = false;  % Set to true to enable adaptive rho

% Quadrotor system matrices (12 states, 4 inputs)
rho_value = 5.0;

% Quadrotor dynamics matrix A (12x12)
A = [1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0245250, 0.0000000, 0.0500000, 0.0000000, 0.0000000, 0.0000000, 0.0002044, 0.0000000;
     0.0000000, 1.0000000, 0.0000000, -0.0245250, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, -0.0002044, 0.0000000, 0.0000000;
     0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0500000, 0.0000000, 0.0000000, 0.0000000;
     0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0250000, 0.0000000, 0.0000000;
     0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0250000, 0.0000000;
     0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0250000;
     0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.9810000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0122625, 0.0000000;
     0.0000000, 0.0000000, 0.0000000, -0.9810000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, -0.0122625, 0.0000000, 0.0000000;
     0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000;
     0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000;
     0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000;
     0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000];

% Input/control matrix B (12x4)
B = [-0.0007069, 0.0007773, 0.0007091, -0.0007795;
     0.0007034, 0.0007747, -0.0007042, -0.0007739;
     0.0052554, 0.0052554, 0.0052554, 0.0052554;
     -0.1720966, -0.1895213, 0.1722891, 0.1893288;
     -0.1729419, 0.1901740, 0.1734809, -0.1907131;
     0.0123423, -0.0045148, -0.0174024, 0.0095748;
     -0.0565520, 0.0621869, 0.0567283, -0.0623632;
     0.0562756, 0.0619735, -0.0563386, -0.0619105;
     0.2102143, 0.2102143, 0.2102143, 0.2102143;
     -13.7677303, -15.1617018, 13.7831318, 15.1463003;
     -13.8353509, 15.2139209, 13.8784751, -15.2570451;
     0.9873856, -0.3611820, -1.3921880, 0.7659845];

% Cost matrices
Q_diag = [100.0, 100.0, 100.0, 4.0, 4.0, 400.0, 4.0, 4.0, 4.0, 2.0408163, 2.0408163, 4.0];
R_diag = [4.0, 4.0, 4.0, 4.0];
Q = diag(Q_diag);
R = diag(R_diag);

N = 20;

% Problem dimensions
nx = size(A, 1);
nu = size(B, 2);

% Affine dynamics term (zeros for linear system)
fdyn = zeros(nx, 1);

% Input constraints
u_min_val = -2.0;
u_max_val = 2.0;

% Set up bounds
infty = 1e17;
x_min = -infty * ones(nx, N);
x_max = infty * ones(nx, N);
u_min = u_min_val * ones(nu, N-1);
u_max = u_max_val * ones(nu, N-1);

% Setup parameters
rho = rho_value;
verbose = 1;  % Set to 1 for verbose output

fprintf('Setting up TinyMPC solver for quadrotor hover...\n');
if ENABLE_ADAPTIVE_RHO
    fprintf('Adaptive rho enabled\n');
else
    fprintf('Adaptive rho disabled\n');
end

% Setup solver
status = tinympc_matlab('setup', A, B, fdyn, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, verbose);

if status ~= 0
    error('Setup failed with status %d', status);
end

% Generate code
fprintf('Generating C code for quadrotor hover...\n');

if ENABLE_ADAPTIVE_RHO
    fprintf('Note: Adaptive rho sensitivity matrices not implemented in MATLAB wrapper\n');
    fprintf('Generating code without sensitivity matrices...\n');
    codegen_status = tinympc_matlab('codegen', 'out', verbose);
else
    fprintf('Generating code without sensitivity matrices...\n');
    codegen_status = tinympc_matlab('codegen', 'out', verbose);
end

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

fprintf('\nQuadrotor hover code generation example completed!\n');
