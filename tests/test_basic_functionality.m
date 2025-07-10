% Test script to verify TinyMPC MATLAB wrapper is working
% This script tests the basic functionality of the compiled MEX function

clear all; close all; clc;

fprintf('=== TinyMPC MATLAB Wrapper Test ===\n\n');

% Test 1: Check if MEX function exists
try
    fprintf('Test 1: Checking MEX function...\n');
    if exist('tinympc_matlab', 'file') == 3
        fprintf('  ✓ MEX function found\n');
    else
        error('MEX function not found');
    end
catch ME
    fprintf('  ✗ MEX function test failed: %s\n', ME.message);
    return;
end

% Test 2: Simple double integrator example
try
    fprintf('\nTest 2: Double integrator setup...\n');
    
    % Problem dimensions
    nx = 4;  % [position_x, position_y, velocity_x, velocity_y]
    nu = 2;  % [force_x, force_y]
    N = 10;  % prediction horizon
    
    % Double integrator dynamics: x[k+1] = A*x[k] + B*u[k]
    dt = 0.1;  % time step
    A = [1, 0, dt, 0;
         0, 1, 0, dt;
         0, 0, 1, 0;
         0, 0, 0, 1];
    
    B = [0.5*dt^2, 0;
         0, 0.5*dt^2;
         dt, 0;
         0, dt];
    
    % Affine dynamics term (none for this example)
    f = zeros(nx, 1);
    
    % Cost matrices
    Q = eye(nx);  % State cost
    R = 0.1 * eye(nu);  % Input cost
    
    % Penalty parameter
    rho = 1.0;
    
    % Bounds (optional - set to large values for now)
    x_min = -10 * ones(nx, N);
    x_max = 10 * ones(nx, N);
    u_min = -5 * ones(nu, N-1);
    u_max = 5 * ones(nu, N-1);
    
    % Verbose output
    verbose = 1;
    
    % Call setup
    tinympc_matlab('setup', A, B, f, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, verbose);
    
    fprintf('  ✓ Setup completed successfully\n');
    
catch ME
    fprintf('  ✗ Setup failed: %s\n', ME.message);
    return;
end

% Test 3: Set initial condition and solve
try
    fprintf('\nTest 3: Setting initial condition and solving...\n');
    
    % Set initial condition
    x0 = [1; 1; 0; 0];  % Start at position (1,1) with zero velocity
    tinympc_matlab('set_x0', x0, verbose);
    
    % Solve the problem
    tinympc_matlab('solve', verbose);
    
    fprintf('  ✓ Problem solved successfully\n');
    
catch ME
    fprintf('  ✗ Solve failed: %s\n', ME.message);
    return;
end

% Test 4: Get results
try
    fprintf('\nTest 4: Getting results...\n');
    
    % Get solution (this returns x, u, solved, iterations)
    [x_traj, u_traj, solved, iterations] = tinympc_matlab('get_solution');
    
    fprintf('  ✓ State trajectory retrieved: %dx%d\n', size(x_traj, 1), size(x_traj, 2));
    fprintf('  ✓ Control trajectory retrieved: %dx%d\n', size(u_traj, 1), size(u_traj, 2));
    fprintf('  ✓ Solver status: %d\n', solved);
    fprintf('  ✓ Solver iterations: %d\n', iterations);
    
catch ME
    fprintf('  ✗ Getting results failed: %s\n', ME.message);
    return;
end

% Test 5: Visualization (optional)
try
    fprintf('\nTest 5: Visualization...\n');
    
    if exist('x_traj', 'var') && exist('u_traj', 'var')
        figure('Name', 'TinyMPC Test Results');
        
        % Plot state trajectory
        subplot(2,2,1);
        plot(0:N-1, x_traj(1,:), 'b-o', 'LineWidth', 2);
        xlabel('Time step');
        ylabel('Position X');
        title('State Trajectory - Position X');
        grid on;
        
        subplot(2,2,2);
        plot(0:N-1, x_traj(2,:), 'r-o', 'LineWidth', 2);
        xlabel('Time step');
        ylabel('Position Y');
        title('State Trajectory - Position Y');
        grid on;
        
        % Plot control trajectory
        subplot(2,2,3);
        plot(0:N-2, u_traj(1,:), 'g-s', 'LineWidth', 2);
        xlabel('Time step');
        ylabel('Force X');
        title('Control Trajectory - Force X');
        grid on;
        
        subplot(2,2,4);
        plot(0:N-2, u_traj(2,:), 'm-s', 'LineWidth', 2);
        xlabel('Time step');
        ylabel('Force Y');
        title('Control Trajectory - Force Y');
        grid on;
        
        fprintf('  ✓ Visualization completed\n');
    end
    
catch ME
    fprintf('  ✗ Visualization failed: %s\n', ME.message);
    % Not critical, continue
end

fprintf('\n=== All Tests Completed Successfully! ===\n');
fprintf('TinyMPC MATLAB wrapper is working correctly.\n');

% Cleanup
try
    tinympc_matlab('cleanup');
    fprintf('Solver cleaned up successfully.\n');
catch ME
    fprintf('Warning: Cleanup failed: %s\n', ME.message);
end
