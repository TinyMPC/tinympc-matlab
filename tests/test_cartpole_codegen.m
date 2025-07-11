% Test code generation for cartpole
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'matlab_wrapper'));

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
    prob = TinyMPC();
    prob.setup(A, B, Q, R, N, 'u_min', u_min, 'u_max', u_max, 'rho', 1.0);
    outdir = 'out';
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    prob.codegen(outdir);
    assert(exist(fullfile(outdir, 'CMakeLists.txt'), 'file') == 2, 'Codegen failed: CMakeLists.txt missing');
    fprintf('test_cartpole_codegen.m: PASSED\n');
catch ME
    if contains(ME.message, 'tinympc_matlab')
        fprintf('test_cartpole_codegen.m: SKIPPED (MEX file not compiled)\n');
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
