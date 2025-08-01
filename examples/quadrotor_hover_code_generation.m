
% Quadrotor Hover Code Generation Example (MATLAB)
clear; close all;

% Add TinyMPC class to path

currentFile = mfilename('fullpath');
[scriptPath, ~, ~] = fileparts(currentFile);
repoRoot = fileparts(scriptPath);
addpath(fullfile(repoRoot, 'src'));
addpath(fullfile(repoRoot, 'build'));

% Enable or disable adaptive rho
ENABLE_ADAPTIVE_RHO = false;   % Set true to enable adaptive rho

% Quadrotor system matrices (12 states, 4 inputs)
rho_value = 5.0;
Adyn = [1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0245250, 0.0000000, 0.0500000, 0.0000000, 0.0000000, 0.0000000, 0.0002044, 0.0000000;
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

% Input matrix
Bdyn = [-0.0007069, 0.0007773, 0.0007091, -0.0007795;
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

Q_diag = [100.0000000, 100.0000000, 100.0000000, 4.0000000, 4.0000000, 400.0000000, ...
          4.0000000, 4.0000000, 4.0000000, 2.0408163, 2.0408163, 4.0000000];
R_diag = [4.0000000, 4.0000000, 4.0000000, 4.0000000];
Q = diag(Q_diag);
R = diag(R_diag);

N = 20;

% Create and set up solver
prob = TinyMPC();

prob.setup(Adyn, Bdyn, Q, R, N, 'rho', rho_value, 'verbose', true, 'adaptive_rho', ENABLE_ADAPTIVE_RHO);

% Set reference trajectory (hover at origin)
Xref = zeros(size(Adyn,1), N);        % nx x N
Uref = zeros(size(Bdyn,2), N-1);      % nu x (N-1)
prob.set_x_ref(Xref);
prob.set_u_ref(Uref);

if ENABLE_ADAPTIVE_RHO
    % Compute cache terms
    [Kinf, Pinf, Quu_inv, AmBKt] = prob.compute_cache_terms();
    % Compute sensitivity matrices
    [dK, dP, dC1, dC2] = prob.compute_sensitivity_autograd();
    % Print matrix norms for inspection
    fprintf('Sensitivity matrix norms: dK=%.6e, dP=%.6e, dC1=%.6e, dC2=%.6e\n', norm(dK), norm(dP), norm(dC1), norm(dC2));
    % Generate code with sensitivity
    prob.codegen_with_sensitivity('out', dK, dP, dC1, dC2);
else
    % Generate code without sensitivity
    prob.codegen('out');
end
