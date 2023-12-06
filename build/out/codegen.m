% Define the input parameters (adjust these values as needed)
nx = 4; % Example value
nu = 1; % Example value
N = 10; % Example value

% Create example data for input parameters (adjust these matrices as needed)
Adyn = [1.0, 0.0, 0.0, 0.0, 0.01, 1.0, 0.0, 0.0, 2.2330083403300767e-5, 0.004466210576510177, 1.0002605176397052, 0.05210579005928538, 7.443037974683548e-8, 2.2330083403300767e-5, 0.01000086835443038, 1.0002605176397052];
Bdyn = [7.468368562730335e-5, 0.014936765390161838, 3.79763323185387e-5, 0.007595596218554721];
Q = [10, 1, 10, 1];
Qf = [10, 1, 10, 1];
R = 1;
rho = 0.1;

x_min = repmat(-5, 1, nx * N);
x_max = repmat(5, 1, nx * N);
u_min = repmat(-5, 1, nu * (N - 1));
u_max = repmat(5, 1, nu * (N - 1));
abs_pri_tol = single(0.001); % Example value
abs_dua_tol = single(0.001); % Example value
max_iters = 100; % Example value
check_termination = 1; % Example value
tinympc_dir = '/Users/elakhyanedumaran/Documents/Research/Copy_of_Test_codegen_final/'; % Example path
output_dir = '/generated_code'; % Example path

% Call the MEX function
result = tinympc_mex(nx, nu, N, Adyn, Bdyn, Q, Qf, R, x_min, x_max, u_min, u_max, rho, abs_pri_tol, abs_dua_tol, max_iters, check_termination, tinympc_dir, output_dir);

% Display the result
disp(['Result: ' num2str(result)]);
