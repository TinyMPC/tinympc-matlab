% test_quadrotor_codegen.m - Test code generation for quadrotor (MATLAB)
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'matlab_wrapper'));
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'examples'));

% Load quadrotor example data (copy from quadrotor_hover_code_generation.m)
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
u_min = -0.5;
u_max = 0.5;
rho_value = 5.0;

try
    prob = TinyMPC(size(Adyn,1), size(Bdyn,2), N, Adyn, Bdyn, Q, R);
    prob.setup('u_min', u_min, 'u_max', u_max, 'rho', rho_value, 'verbose', false);
    
    test_dir = 'out';
    if ~exist(test_dir, 'dir')
        mkdir(test_dir);
    end
    prob.codegen(test_dir);
    
    % Check for expected files (based on MATLAB structure)
    expected_files = { 'CMakeLists.txt', ...
        fullfile('src','tiny_main.cpp'), fullfile('src','tiny_data.cpp'), ...
        fullfile('tinympc','tiny_data.hpp'), fullfile('tinympc','admm.hpp'), fullfile('tinympc','codegen.hpp') };
    
    fprintf('Checking generated files in: %s\n', test_dir);
    for i = 1:length(expected_files)
        f = fullfile(test_dir, expected_files{i});
        if exist(f, 'file') == 2 || exist(f, 'file') == 7
            fprintf('  ✓ %s\n', expected_files{i});
        else
            fprintf('  ✗ Missing: %s\n', expected_files{i});
        end
    end
    
    fprintf('test_quadrotor_codegen.m: COMPLETED (check output above)\n');
    
catch ME
    if contains(ME.message, 'tinympc_matlab')
        fprintf('test_quadrotor_codegen.m: SKIPPED (MEX file not compiled)\n');
        fprintf('To test code generation, first compile the MEX file using compile_tinympc_matlab.m\n');
    else
        fprintf('test_quadrotor_codegen.m: ERROR - %s\n', ME.message);
        rethrow(ME);
    end
end

function list_files_recursive(dir_path, prefix)
    items = dir(dir_path);
    for i = 1:length(items)
        if ~strcmp(items(i).name, '.') && ~strcmp(items(i).name, '..')
            full_path = fullfile(dir_path, items(i).name);
            if items(i).isdir
                fprintf('%s%s/\n', prefix, items(i).name);
                list_files_recursive(full_path, [prefix, '  ']);
            else
                fprintf('%s%s\n', prefix, items(i).name);
            end
        end
    end
end
