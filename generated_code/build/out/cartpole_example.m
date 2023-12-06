% Set initial state
x0 = single([0.0; 0; 0.1; 0]);
xref = single(zeros(1,40));
Xref = single(zeros(4,10));
% Simulation parameters
N = 200;
cmd_2 = 'set_xref';
tinympc_wrapper_mex(cmd_2, xref);
cmd_get_Adyn = 'get_Adyn';
A_dyn = zeros(4, 4,'single');
tinympc_wrapper_mex(cmd_get_Adyn, A_dyn);
A_dyn
cmd_get_Bdyn = 'get_Bdyn';
B_dyn = zeros(4, 1,'single');
tinympc_wrapper_mex(cmd_get_Bdyn, B_dyn);
for k = 1:N
    fprintf('tracking error at step %2d: %.4f\n', k, norm(x0 - Xref(:, 1)));
    
    % Update measurement
    cmd_1 = 'set_x';
    tinympc_wrapper_mex(cmd_1, x0);

    % Reset dual variables
    cmd_3 = 'reset_dual_variables';
    tinympc_wrapper_mex(cmd_3);

    % Solve MPC problem
    cmd_4 = 'call_tiny_solve';
    tinympc_wrapper_mex(cmd_4);
    % Call get_u function
    cmd_get_u = 'get_u';
    u_soln = zeros(1, 10 - 1,'single'); % Assuming NHORIZON is defined
    tinympc_wrapper_mex(cmd_get_u, u_soln);
    u_soln
    % Simulate forward
    x1 = A_dyn * x0 + B_dyn * u_soln(:, 1);
    x0 = single(x1);
    x0
end
