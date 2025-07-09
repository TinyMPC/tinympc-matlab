function test_with_library()
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
    
    function main()
        fprintf('=== TinyMPC MATLAB Integration Test ===\n\n');
        
        % Default paths (modify these based on your build system)
        default_lib_path = 'tinympc_matlab.dll';  % Windows
        default_header_path = 'wrapper.hpp';
        
        % Parse input arguments (if any)
        if nargin >= 2
            lib_path = varargin{2};
        else
            lib_path = default_lib_path;
        end
        
        if nargin >= 4
            header_path = varargin{4};
        else
            header_path = default_header_path;
        end
        
        try
            % Test 1: Library loading
            fprintf('Test 1: Loading library...\n');
            test_library_loading(lib_path, header_path);
            fprintf('✓ Library loading: PASSED\n\n');
            
            % Test 2: Basic setup and solve
            fprintf('Test 2: Basic setup and solve...\n');
            test_basic_solve();
            fprintf('✓ Basic solve: PASSED\n\n');
            
            % Test 3: Constraint handling
            fprintf('Test 3: Constraint handling...\n');
            test_constraints();
            fprintf('✓ Constraint handling: PASSED\n\n');
            
            % Test 4: Reference tracking
            fprintf('Test 4: Reference tracking...\n');
            test_references();
            fprintf('✓ Reference tracking: PASSED\n\n');
            
            % Test 5: Code generation
            fprintf('Test 5: Code generation...\n');
            test_codegen();
            fprintf('✓ Code generation: PASSED\n\n');
            
            fprintf('🎉 All integration tests passed!\n');
            
        catch ME
            fprintf('❌ Integration test failed: %s\n', ME.message);
            fprintf('Stack trace:\n');
            for i = 1:length(ME.stack)
                fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
            end
        end
    end

    function test_library_loading(lib_path, header_path)
        % Test library loading functionality
        
        % Simple system for testing
        A = eye(2);
        B = [1; 0];
        Q = eye(2);
        R = 1;
        
        solver = TinyMPC(2, 1, 5, A, B, Q, R, 'verbose', true);
        
        % Test library loading
        success = solver.load_library(lib_path, header_path);
        assert(success, 'Failed to load library');
        
        fprintf('  - Library loaded successfully\n');
    end

    function test_basic_solve()
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
        solver = TinyMPC(nx, nu, N, A, B, Q, R, 'rho', 1.0, 'verbose', true);
        
        % Setup solver
        success = solver.setup();
        assert(success, 'Solver setup failed');
        fprintf('  - Solver setup: OK\n');
        
        % Set initial state
        x0 = [0.1; 0; 0.05; 0];  % Small perturbation from equilibrium
        success = solver.set_initial_state(x0);
        assert(success, 'Failed to set initial state');
        fprintf('  - Initial state set: OK\n');
        
        % Solve
        success = solver.solve();
        assert(success, 'Solve failed');
        fprintf('  - Solve completed: OK\n');
        
        % Get solution
        [x_traj, u_traj] = solver.get_solution();
        
        % Validate solution dimensions
        assert(size(x_traj, 1) == nx && size(x_traj, 2) == N, 'Wrong x_traj dimensions');
        assert(size(u_traj, 1) == nu && size(u_traj, 2) == N-1, 'Wrong u_traj dimensions');
        fprintf('  - Solution retrieved: OK\n');
        
        % Check solver info
        [status, iterations] = solver.get_solver_info();
        fprintf('  - Solver status: %d, iterations: %d\n', status, iterations);
        
        % Basic sanity check: initial state should match
        assert(norm(x_traj(:,1) - x0) < 1e-10, 'Initial state mismatch');
        fprintf('  - Solution validation: OK\n');
    end

    function test_constraints()
        % Test constraint handling
        
        % Simple 2D double integrator
        A = [1, 1; 0, 1];
        B = [0.5; 1];
        Q = diag([10, 1]);
        R = 1;
        
        nx = 2; nu = 1; N = 8;
        
        solver = TinyMPC(nx, nu, N, A, B, Q, R, 'verbose', true);
        solver.setup();
        
        % Set constraints
        x_min = [-2; -1] * ones(1, N);  % Position and velocity limits
        x_max = [2; 1] * ones(1, N);
        u_min = -0.5 * ones(nu, N-1);   % Input limits
        u_max = 0.5 * ones(nu, N-1);
        
        success = solver.set_bounds(x_min, x_max, u_min, u_max);
        assert(success, 'Failed to set constraints');
        fprintf('  - Constraints set: OK\n');
        
        % Set initial state that requires constraint handling
        x0 = [1.5; 0.8];  % Close to constraints
        solver.set_initial_state(x0);
        
        % Solve
        solver.solve();
        [x_traj, u_traj] = solver.get_solution();
        
        % Check constraint satisfaction
        assert(all(x_traj(1,:) >= x_min(1,:) - 1e-6), 'Position lower bound violated');
        assert(all(x_traj(1,:) <= x_max(1,:) + 1e-6), 'Position upper bound violated');
        assert(all(x_traj(2,:) >= x_min(2,:) - 1e-6), 'Velocity lower bound violated');
        assert(all(x_traj(2,:) <= x_max(2,:) + 1e-6), 'Velocity upper bound violated');
        assert(all(u_traj >= u_min - 1e-6), 'Input lower bound violated');
        assert(all(u_traj <= u_max + 1e-6), 'Input upper bound violated');
        
        fprintf('  - Constraint satisfaction verified: OK\n');
    end

    function test_references()
        % Test reference tracking
        
        % 2D system
        A = [0.9, 0.1; 0, 0.8];
        B = [0; 1];
        Q = diag([10, 1]);
        R = 1;
        
        nx = 2; nu = 1; N = 15;
        
        solver = TinyMPC(nx, nu, N, A, B, Q, R, 'verbose', true);
        solver.setup();
        
        % Create sinusoidal reference
        t = (0:N-1) / (N-1) * 2 * pi;
        x_ref = [sin(t); 0.5*cos(t)];  % nx x N
        u_ref = 0.1 * sin(t(1:end-1));  % nu x (N-1)
        u_ref = reshape(u_ref, nu, N-1);
        
        % Set references
        success = solver.set_reference_trajectory(x_ref, u_ref);
        assert(success, 'Failed to set references');
        fprintf('  - References set: OK\n');
        
        % Start from zero
        x0 = zeros(nx, 1);
        solver.set_initial_state(x0);
        
        % Solve
        solver.solve();
        [x_traj, u_traj] = solver.get_solution();
        
        % Check that trajectory follows reference reasonably
        tracking_error = norm(x_traj - x_ref, 'fro');
        fprintf('  - Tracking error: %.4f\n', tracking_error);
        
        % Error should be reasonable (this is problem-dependent)
        assert(tracking_error < 10, 'Tracking error too large');
        fprintf('  - Reference tracking verified: OK\n');
    end

    function test_codegen()
        % Test code generation
        
        % Simple system
        A = [1.1, 0.1; 0, 0.9];
        B = [0; 1];
        Q = eye(2);
        R = 1;
        
        nx = 2; nu = 1; N = 5;
        
        solver = TinyMPC(nx, nu, N, A, B, Q, R, 'verbose', true);
        solver.setup();
        
        % Create temporary directory for code generation
        output_dir = fullfile(tempdir, 'tinympc_codegen_test');
        if exist(output_dir, 'dir')
            rmdir(output_dir, 's');
        end
        
        % Generate code
        success = solver.codegen(output_dir);
        assert(success, 'Code generation failed');
        fprintf('  - Code generation completed: OK\n');
        
        % Check that essential files were created
        essential_files = {
            'CMakeLists.txt',
            'src/tiny_data.cpp', 
            'tinympc/tiny_data.hpp'
        };
        
        for i = 1:length(essential_files)
            file_path = fullfile(output_dir, essential_files{i});
            assert(exist(file_path, 'file') == 2, ['Missing file: ' essential_files{i}]);
        end
        
        fprintf('  - Generated files verified: OK\n');
        
        % Cleanup
        if exist(output_dir, 'dir')
            rmdir(output_dir, 's');
        end
    end

    % Run the main test function
    main();
end
