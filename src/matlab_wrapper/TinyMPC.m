classdef TinyMPC < handle
    properties (Access = private)
        solver_ptr = []; % Pointer to the C++ solver
        is_setup = false; % Flag to track if solver is setup
        lib_name = 'tinympc_matlab'; % Default library name
    end
    
    properties (Access = public)
        % Problem dimensions
        nx = 0; % State dimension
        nu = 0; % Input dimension
        N = 0; % Prediction horizon
        
        % Problem matrices
        A = []; % State transition matrix
        B = []; % Input matrix
        f = []; % Affine term (optional, defaults to zeros)
        Q = []; % State cost matrix
        R = []; % Input cost matrix
        
        % Constraint bounds (optional)
        x_min = []; % State lower bounds (nx x N)
        x_max = []; % State upper bounds (nx x N)
        u_min = []; % Input lower bounds (nu x N-1)
        u_max = []; % Input upper bounds (nu x N-1)
        
        % Solver parameters
        rho = 1.0; % ADMM penalty parameter
        verbose = true; % Verbose output flag
    end
    
    methods
        function obj = TinyMPC(nx, nu, N, A, B, Q, R, varargin)
            % Constructor for TinyMPC class
            % 
            % Usage:
            %   solver = TinyMPC(nx, nu, N, A, B, Q, R)
            %   solver = TinyMPC(nx, nu, N, A, B, Q, R, 'rho', 2.0, 'verbose', true)
            %
            % Required arguments:
            %   nx - State dimension
            %   nu - Input dimension  
            %   N - Prediction horizon
            %   A - State transition matrix (nx x nx)
            %   B - Input matrix (nx x nu)
            %   Q - State cost matrix (nx x nx)
            %   R - Input cost matrix (nu x nu)
            %
            % Optional arguments (name-value pairs):
            %   'f' - Affine term (nx x 1), defaults to zeros
            %   'rho' - ADMM penalty parameter, defaults to 1.0
            %   'verbose' - Verbose output flag, defaults to true
            %   'lib_name' - Library name, defaults to 'tinympc_matlab'
            
            if nargin < 7
                error('TinyMPC requires at least 7 arguments: nx, nu, N, A, B, Q, R');
            end
            
            % Parse optional arguments
            p = inputParser;
            addParameter(p, 'f', zeros(nx, 1), @(x) isnumeric(x) && size(x,1) == nx && size(x,2) == 1);
            addParameter(p, 'rho', 1.0, @(x) isnumeric(x) && x > 0);
            addParameter(p, 'verbose', true, @islogical);
            addParameter(p, 'lib_name', 'tinympc_matlab', @ischar);
            parse(p, varargin{:});
            
            % Store problem dimensions
            obj.nx = nx;
            obj.nu = nu;
            obj.N = N;
            
            % Store problem matrices
            obj.A = A;
            obj.B = B;
            obj.f = p.Results.f;
            obj.Q = Q;
            obj.R = R;
            
            % Store parameters
            obj.rho = p.Results.rho;
            obj.verbose = p.Results.verbose;
            obj.lib_name = p.Results.lib_name;
            
            % Validate dimensions
            obj.validate_dimensions();
            
            if obj.verbose
                fprintf('TinyMPC initialized with dimensions: nx=%d, nu=%d, N=%d\n', nx, nu, N);
            end
        end
        
        function success = setup(obj)
            % Setup the TinyMPC solver with problem data
            %
            % Returns:
            %   success - true if setup was successful, false otherwise
            
            if obj.is_setup
                warning('Solver already setup. Call reset() first to re-setup.');
                success = false;
                return;
            end
            
            try
                % Ensure library is loaded
                obj.ensure_library_loaded();
                
                % Flatten matrices for C interface (column-major order)
                A_flat = obj.A(:);
                B_flat = obj.B(:);
                f_flat = obj.f(:);
                Q_flat = obj.Q(:);
                R_flat = obj.R(:);
                
                % Create solver pointer placeholder
                solver_ptr_ptr = libpointer('voidPtrPtr');
                
                % Call C setup function
                status = calllib(obj.lib_name, 'tiny_setup_matlab', ...
                    solver_ptr_ptr, A_flat, B_flat, f_flat, Q_flat, R_flat, ...
                    obj.rho, obj.nx, obj.nu, obj.N, obj.verbose);
                
                if status == 0
                    obj.solver_ptr = solver_ptr_ptr.Value;
                    obj.is_setup = true;
                    success = true;
                    
                    if obj.verbose
                        fprintf('TinyMPC solver setup successful\n');
                    end
                else
                    success = false;
                    error('TinyMPC solver setup failed with status: %d', status);
                end
                
            catch ME
                success = false;
                if obj.verbose
                    fprintf('Error in setup: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function success = set_initial_state(obj, x0)
            % Set the initial state for the MPC problem
            %
            % Arguments:
            %   x0 - Initial state vector (nx x 1)
            %
            % Returns:
            %   success - true if successful, false otherwise
            
            obj.check_setup();
            
            if length(x0) ~= obj.nx
                error('Initial state x0 must have length nx = %d', obj.nx);
            end
            
            try
                calllib(obj.lib_name, 'set_x0', x0(:), obj.verbose);
                success = true;
            catch ME
                success = false;
                if obj.verbose
                    fprintf('Error setting initial state: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function success = set_reference_trajectory(obj, x_ref, u_ref)
            % Set reference trajectories for state and input
            %
            % Arguments:
            %   x_ref - State reference trajectory (nx x N)
            %   u_ref - Input reference trajectory (nu x N-1)
            %
            % Returns:
            %   success - true if successful, false otherwise
            
            obj.check_setup();
            
            try
                if ~isempty(x_ref)
                    if size(x_ref, 1) ~= obj.nx || size(x_ref, 2) ~= obj.N
                        error('x_ref must be nx x N = %d x %d', obj.nx, obj.N);
                    end
                    calllib(obj.lib_name, 'set_x_ref', x_ref(:), obj.verbose);
                end
                
                if ~isempty(u_ref)
                    if size(u_ref, 1) ~= obj.nu || size(u_ref, 2) ~= (obj.N-1)
                        error('u_ref must be nu x (N-1) = %d x %d', obj.nu, obj.N-1);
                    end
                    calllib(obj.lib_name, 'set_u_ref', u_ref(:), obj.verbose);
                end
                
                success = true;
            catch ME
                success = false;
                if obj.verbose
                    fprintf('Error setting reference trajectory: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function success = set_bounds(obj, x_min, x_max, u_min, u_max)
            % Set bound constraints
            %
            % Arguments:
            %   x_min - State lower bounds (nx x N)
            %   x_max - State upper bounds (nx x N)
            %   u_min - Input lower bounds (nu x N-1)
            %   u_max - Input upper bounds (nu x N-1)
            %
            % Returns:
            %   success - true if successful, false otherwise
            
            obj.check_setup();
            
            % Validate dimensions
            if size(x_min, 1) ~= obj.nx || size(x_min, 2) ~= obj.N
                error('x_min must be nx x N = %d x %d', obj.nx, obj.N);
            end
            if size(x_max, 1) ~= obj.nx || size(x_max, 2) ~= obj.N
                error('x_max must be nx x N = %d x %d', obj.nx, obj.N);
            end
            if size(u_min, 1) ~= obj.nu || size(u_min, 2) ~= (obj.N-1)
                error('u_min must be nu x (N-1) = %d x %d', obj.nu, obj.N-1);
            end
            if size(u_max, 1) ~= obj.nu || size(u_max, 2) ~= (obj.N-1)
                error('u_max must be nu x (N-1) = %d x %d', obj.nu, obj.N-1);
            end
            
            try
                calllib(obj.lib_name, 'set_bound_constraints', ...
                    x_min(:), x_max(:), u_min(:), u_max(:), obj.verbose);
                
                % Store bounds for reference
                obj.x_min = x_min;
                obj.x_max = x_max;
                obj.u_min = u_min;
                obj.u_max = u_max;
                
                success = true;
            catch ME
                success = false;
                if obj.verbose
                    fprintf('Error setting bounds: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function success = solve(obj)
            % Solve the MPC problem
            %
            % Returns:
            %   success - true if solve was successful, false otherwise
            
            obj.check_setup();
            
            try
                calllib(obj.lib_name, 'call_tiny_solve', obj.verbose);
                success = true;
            catch ME
                success = false;
                if obj.verbose
                    fprintf('Error solving: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function [x_traj, u_traj] = get_solution(obj)
            % Get the solution trajectories
            %
            % Returns:
            %   x_traj - State trajectory (nx x N)
            %   u_traj - Input trajectory (nu x N-1)
            
            obj.check_setup();
            
            try
                % Get state trajectory
                x_soln = zeros(obj.nx * obj.N, 1);
                calllib(obj.lib_name, 'get_x', x_soln, obj.verbose);
                x_traj = reshape(x_soln, obj.nx, obj.N);
                
                % Get input trajectory
                u_soln = zeros(obj.nu * (obj.N-1), 1);
                calllib(obj.lib_name, 'get_u', u_soln, obj.verbose);
                u_traj = reshape(u_soln, obj.nu, obj.N-1);
                
            catch ME
                if obj.verbose
                    fprintf('Error getting solution: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function [status, iterations] = get_solver_info(obj)
            % Get solver status and iteration count
            %
            % Returns:
            %   status - Solver status (0 = solved, 1 = unsolved, -1 = error)
            %   iterations - Number of iterations used
            
            obj.check_setup();
            
            try
                status = calllib(obj.lib_name, 'get_solver_status', obj.verbose);
                iterations = calllib(obj.lib_name, 'get_solver_iterations', obj.verbose);
            catch ME
                if obj.verbose
                    fprintf('Error getting solver info: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function success = reset_dual_variables(obj)
            % Reset ADMM dual variables to zero
            %
            % Returns:
            %   success - true if successful, false otherwise
            
            obj.check_setup();
            
            try
                calllib(obj.lib_name, 'reset_dual_variables', obj.verbose);
                success = true;
            catch ME
                success = false;
                if obj.verbose
                    fprintf('Error resetting dual variables: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function success = codegen(obj, output_dir)
            % Generate standalone C++ code
            %
            % Arguments:
            %   output_dir - Directory to save generated code
            %
            % Returns:
            %   success - true if successful, false otherwise
            
            obj.check_setup();
            
            try
                status = calllib(obj.lib_name, 'tiny_codegen_matlab', ...
                    obj.solver_ptr, output_dir, obj.verbose);
                success = (status == 0);
                
                if success && obj.verbose
                    fprintf('Code generation completed successfully in: %s\n', output_dir);
                end
                
            catch ME
                success = false;
                if obj.verbose
                    fprintf('Error in code generation: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function success = load_library(obj, lib_path, header_path)
            % Load the TinyMPC MATLAB library
            %
            % Arguments:
            %   lib_path - Path to the compiled library (.dll, .so, or .dylib)
            %   header_path - Path to the header file for library interface
            %
            % Returns:
            %   success - true if successful, false otherwise
            
            try
                if libisloaded(obj.lib_name)
                    unloadlibrary(obj.lib_name);
                end
                
                loadlibrary(lib_path, header_path, 'alias', obj.lib_name);
                success = true;
                
                if obj.verbose
                    fprintf('Library %s loaded successfully\n', obj.lib_name);
                    % libfunctions(obj.lib_name, '-full'); % Uncomment to see available functions
                end
                
            catch ME
                success = false;
                if obj.verbose
                    fprintf('Error loading library: %s\n', ME.message);
                end
                rethrow(ME);
            end
        end
        
        function reset(obj)
            % Reset the solver (cleanup current setup)
            
            if obj.is_setup
                % Note: In a full implementation, we'd call a cleanup function here
                % For now, just reset the flag
                obj.solver_ptr = [];
                obj.is_setup = false;
                
                if obj.verbose
                    fprintf('TinyMPC solver reset\n');
                end
            end
        end
        
        function delete(obj)
            % Destructor - cleanup when object is destroyed
            obj.reset();
        end
    end
    
    methods (Access = private)
        function validate_dimensions(obj)
            % Validate that all matrices have correct dimensions
            
            if size(obj.A, 1) ~= obj.nx || size(obj.A, 2) ~= obj.nx
                error('Matrix A must be nx x nx = %d x %d', obj.nx, obj.nx);
            end
            
            if size(obj.B, 1) ~= obj.nx || size(obj.B, 2) ~= obj.nu
                error('Matrix B must be nx x nu = %d x %d', obj.nx, obj.nu);
            end
            
            if size(obj.Q, 1) ~= obj.nx || size(obj.Q, 2) ~= obj.nx
                error('Matrix Q must be nx x nx = %d x %d', obj.nx, obj.nx);
            end
            
            if size(obj.R, 1) ~= obj.nu || size(obj.R, 2) ~= obj.nu
                error('Matrix R must be nu x nu = %d x %d', obj.nu, obj.nu);
            end
            
            if size(obj.f, 1) ~= obj.nx || size(obj.f, 2) ~= 1
                error('Vector f must be nx x 1 = %d x 1', obj.nx);
            end
        end
        
        function check_setup(obj)
            % Check if solver is setup, throw error if not
            if ~obj.is_setup
                error('Solver not setup. Call setup() first.');
            end
        end
        
        function ensure_library_loaded(obj)
            % Ensure the library is loaded
            if ~libisloaded(obj.lib_name)
                error(['Library %s not loaded. Call load_library() first with the path to ' ...
                       'your compiled TinyMPC MATLAB library.'], obj.lib_name);
            end
        end
    end
end