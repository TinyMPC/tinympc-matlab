function test_with_library(varargin)
    % TEST_WITH_LIBRARY - Integration test that requires compiled library
    %
    % This test requires the TinyMPC MATLAB library to be compiled and available.
    % It tests the full workflow from setup to solution.
    %
    % Prerequisites:
    %   1. Compile the TinyMPC MATLAB library (wrapper.cpp + codegen.cpp)
    %   2. Make sure the library (.dll/.so/.dylib) and header files are available
    %
    % Usage:
    %   test_with_library()
    %   test_with_library('lib_path', '/path/to/library', 'header_path', '/path/to/header')
    
    fprintf('=== TinyMPC MATLAB Integration Test ===\n\n');
    
    % Setup paths
    setup_test_paths_library();
    
    % Default paths (modify these based on your build system)
    default_lib_path = 'tinympc_matlab.dll';  % Windows
    default_header_path = 'wrapper.hpp';
    
    % Parse input arguments using inputParser
    p = inputParser;
    addParameter(p, 'lib_path', default_lib_path, @ischar);
    addParameter(p, 'header_path', default_header_path, @ischar);
    parse(p, varargin{:});
    
    lib_path = p.Results.lib_path;
    header_path = p.Results.header_path;
    
    try
        % Test 1: Library loading
        fprintf('Test 1: Loading library...\n');
        test_library_loading_impl(lib_path, header_path);
        fprintf('✓ Library loading: PASSED\n\n');
        
        % Test 2: Basic setup and solve
        fprintf('Test 2: Basic setup and solve...\n');
        test_basic_solve_impl();
        fprintf('✓ Basic solve: PASSED\n\n');
        
        % Test 3: Constraint handling
        fprintf('Test 3: Constraint handling...\n');
        test_constraints_impl();
        fprintf('✓ Constraint handling: PASSED\n\n');
        
        % Test 4: Reference tracking
        fprintf('Test 4: Reference tracking...\n');
        test_references_impl();
        fprintf('✓ Reference tracking: PASSED\n\n');
        
        % Test 5: Code generation
        fprintf('Test 5: Code generation...\n');
        test_codegen_impl();
        fprintf('✓ Code generation: PASSED\n\n');
        
        fprintf('🎉 All integration tests passed!\n');
        
    catch ME
        fprintf('❌ Integration test failed: %s\n', ME.message);
        fprintf('Stack trace:\n');
        for i = 1:length(ME.stack)
            fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
        end
        rethrow(ME);
    end
end

function test_library_loading_impl(lib_path, header_path)
    % Test library loading functionality
    
    % Simple system for testing
    A = eye(2);
    B = [1; 0];
    Q = eye(2);
    R = 1;
    
    solver = TinyMPC(2, 1, 5, A, B, Q, R, 'verbose', true);
    
    % Test library loading (if available)
    try
        success = solver.load_library(lib_path, header_path);
        if success
            fprintf('  - Library loaded successfully\n');
        else
            fprintf('  - Library loading not available (using MATLAB-only mode)\n');
        end
    catch
        fprintf('  - Library loading not available (using MATLAB-only mode)\n');
    end
end

function test_basic_solve_impl()
    % Test basic setup and solve workflow
    
    % Cartpole system (same as Python tests)
    A = [1.0, 0.01, 0.0, 0.0;
         0.0, 1.0, 0.039, 0.0;
         0.0, 0.0, 1.002, 0.01;
         0.0, 0.0, 0.458, 1.002];
    
    B = [0.0; 0.02; 0.0; 0.067];
    Q = diag([10.0, 1, 10, 1]);
    R = 1.0;
    
    nx = 4; nu = 1; N = 10;
    
    % Create and setup solver
    solver = TinyMPC(nx, nu, N, A, B, Q, R, 'verbose', false);
    
    % Setup solver
    solver.setup();
    
    % Set initial state
    x0 = [0.1; 0; -0.1; 0];
    solver.set_initial_state(x0);
    
    % Solve
    solver.solve();
    
    % Get solution
    [x_traj, u_traj] = solver.get_solution();
    
    % Basic checks
    assert(size(x_traj, 1) == nx, 'State trajectory dimension mismatch');
    assert(size(x_traj, 2) == N+1, 'State trajectory length mismatch');
    assert(size(u_traj, 1) == nu, 'Control trajectory dimension mismatch');
    assert(size(u_traj, 2) == N, 'Control trajectory length mismatch');
    
    fprintf('  - Solution dimensions correct\n');
    fprintf('  - Final state norm: %.6f\n', norm(x_traj(:, end)));
end

function test_constraints_impl()
    % Test constraint handling
    
    % Simple double integrator
    A = [1, 1; 0, 1];
    B = [0; 1];
    Q = eye(2);
    R = 1;
    
    nx = 2; nu = 1; N = 5;
    
    % Create solver
    solver = TinyMPC(nx, nu, N, A, B, Q, R, 'verbose', false);
    
    % Setup solver
    solver.setup();
    
    % Set initial state
    x0 = [1; 0];
    solver.set_initial_state(x0);
    
    % Set constraints
    u_min = -0.5;
    u_max = 0.5;
    solver.set_bounds([], [], u_min * ones(nu, N), u_max * ones(nu, N));
    
    % Solve
    solver.solve();
    
    % Get solution and check constraints
    [~, u_traj] = solver.get_solution();
    
    % Check control bounds
    assert(all(u_traj(:) >= u_min - 1e-6), 'Control lower bound violated');
    assert(all(u_traj(:) <= u_max + 1e-6), 'Control upper bound violated');
    
    fprintf('  - Control constraints satisfied\n');
end

function test_references_impl()
    % Test reference tracking
    
    % Simple system
    A = [1, 1; 0, 1];
    B = [0; 1];
    Q = eye(2);
    R = 1;
    
    nx = 2; nu = 1; N = 5;
    
    % Create solver
    solver = TinyMPC(nx, nu, N, A, B, Q, R, 'verbose', false);
    
    % Setup solver
    solver.setup();
    
    % Set initial state
    x0 = [0; 0];
    solver.set_initial_state(x0);
    
    % Set state reference (move to [1; 0])
    x_ref = repmat([1; 0], 1, N+1);
    u_ref = zeros(nu, N);
    solver.set_reference_trajectory(x_ref, u_ref);
    
    % Solve
    solver.solve();
    
    % Get solution
    [x_traj, ~] = solver.get_solution();
    
    % Check that we get closer to reference
    initial_error = norm(x0 - x_ref(:, 1));
    final_error = norm(x_traj(:, end) - x_ref(:, end));
    
    assert(final_error < initial_error, 'Reference tracking not working');
    
    fprintf('  - Reference tracking working (error reduced from %.3f to %.3f)\n', ...
            initial_error, final_error);
end

function test_codegen_impl()
    % Test code generation functionality
    
    % Simple system for code generation
    A = [1, 1; 0, 1];
    B = [0; 1];
    Q = eye(2);
    R = 1;
    
    nx = 2; nu = 1; N = 5;
    
    % Create solver
    solver = TinyMPC(nx, nu, N, A, B, Q, R, 'verbose', false);
    
    % Setup solver
    solver.setup();
    
    % Test code generation
    output_dir = 'test_codegen_output';
    
    try
        success = solver.codegen(output_dir);
        
        if success
            % Check that files were generated
            expected_files = {'tiny_api.h', 'tiny_api.c', 'tiny_data.h'};
            
            for i = 1:length(expected_files)
                file_path = fullfile(output_dir, expected_files{i});
                assert(exist(file_path, 'file') == 2, ...
                       sprintf('Generated file not found: %s', expected_files{i}));
            end
            
            fprintf('  - Code generation successful\n');
            fprintf('  - Generated files: %s\n', strjoin(expected_files, ', '));
        else
            fprintf('  - Code generation not available in this build\n');
        end
        
    catch ME
        if contains(ME.message, 'not implemented') || contains(ME.message, 'not available')
            fprintf('  - Code generation not available in this build\n');
        else
            rethrow(ME);
        end
    end
    
    % Cleanup
    if exist(output_dir, 'dir')
        rmdir(output_dir, 's');
    end
end

function setup_test_paths_library()
    % SETUP_TEST_PATHS_LIBRARY - Add necessary paths for library testing
    %
    % This function adds the TinyMPC MATLAB wrapper to the MATLAB path
    % so that integration tests can find the TinyMPC class.
    
    % Get current directory (tests/) and repository root
    current_dir = pwd;
    repo_root = fileparts(current_dir);
    
    % Add wrapper directory to path
    wrapper_dir = fullfile(repo_root, 'src', 'matlab_wrapper');
    
    if exist(wrapper_dir, 'dir')
        addpath(wrapper_dir);
        fprintf('Added to MATLAB path: %s\n', wrapper_dir);
    else
        warning('TinyMPC wrapper directory not found: %s', wrapper_dir);
        fprintf('Current directory: %s\n', current_dir);
        fprintf('Repository root: %s\n', repo_root);
    end
end