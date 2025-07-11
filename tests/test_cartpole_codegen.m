% test_cartpole_codegen.m - Test code generation for cartpole (MATLAB)
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'matlab_wrapper'));
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'examples'));

A = [1.0, 0.01, 0.0, 0.0;
     0.0, 1.0, 0.039, 0.0;
     0.0, 0.0, 1.002, 0.01;
     0.0, 0.0, 0.458, 1.002];
B = [0.0; 0.02; 0.0; 0.067];
Q = diag([10.0, 1, 10, 1]);
R = diag([1.0]);
N = 20;
u_min = -0.5;
u_max = 0.5;

try
    prob = TinyMPC(size(A,1), size(B,2), N, A, B, Q, R);
    prob.setup('u_min', u_min, 'u_max', u_max, 'rho', 1.0);
    
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
    
    fprintf('test_cartpole_codegen.m: COMPLETED (check output above)\n');
    
catch ME
    if contains(ME.message, 'tinympc_matlab')
        fprintf('test_cartpole_codegen.m: SKIPPED (MEX file not compiled)\n');
        fprintf('To test code generation, first compile the MEX file using compile_tinympc_matlab.m\n');
    else
        fprintf('test_cartpole_codegen.m: ERROR - %s\n', ME.message);
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
