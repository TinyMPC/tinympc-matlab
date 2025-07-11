classdef TinyMPC < handle
    properties (Access = private)
        is_setup = false; % Flag to track if solver is setup
    end
    
    properties (Access = public)
        % Problem dimensions
        nx = 0; % State dimension
        nu = 0; % Input dimension
        N = 0; % Prediction horizon
        
        % Problem matrices (required)
        A = []; % State transition matrix (nx x nx)
        B = []; % Input matrix (nx x nu)
        Q = []; % State cost matrix (nx x nx)
        R = []; % Input cost matrix (nu x nu)
        
        % Solver parameters
        rho = 1.0; % ADMM penalty parameter
        settings = struct(); % Solver settings
        
        % Sensitivity matrices for adaptive rho
        dK = []; % Derivative of feedback gain w.r.t. rho
        dP = []; % Derivative of value function w.r.t. rho
        dC1 = []; % Derivative of first cache matrix w.r.t. rho
        dC2 = []; % Derivative of second cache matrix w.r.t. rho
        
        % Constraint bounds
        x_min = []; % State lower bounds
        x_max = []; % State upper bounds  
        u_min = []; % Input lower bounds
        u_max = []; % Input upper bounds
    end
    
    methods
        function obj = TinyMPC()
            % Constructor for TinyMPC class (empty constructor like Python)
            % 
            % Usage:
            %   solver = TinyMPC();
            %   solver.setup(A, B, Q, R, N, ...);
            %
            % Notes:
            %   - Use setup() method to configure problem matrices, constraints and solver options
            %   - Call setup() before using solve()
            
            % Initialize default settings
            obj.settings = struct();
            obj.settings.abs_pri_tol = 1e-4;
            obj.settings.abs_dua_tol = 1e-4;
            obj.settings.max_iter = 100;
            obj.settings.check_termination = 1;
            obj.settings.en_state_bound = 0;
            obj.settings.en_input_bound = 0;
            obj.settings.adaptive_rho = 0;
            obj.settings.adaptive_rho_min = 0.1;
            obj.settings.adaptive_rho_max = 10.0;
            obj.settings.adaptive_rho_enable_clipping = 1;
        end
        
        function setup(obj, A, B, Q, R, N, varargin)
            % Setup the TinyMPC solver (Python-compatible interface)
            %
            % Usage:
            %   obj.setup(A, B, Q, R, N)  % Required arguments
            %   obj.setup(A, B, Q, R, N, 'rho', 1.5, 'verbose', true)
            %   obj.setup(A, B, Q, R, N, 'x_min', -2, 'x_max', 2, 'rho', 1.0)
            %   obj.setup(A, B, Q, R, N, 'fdyn', f_vec, 'u_min', u_min_array, ...)
            %
            % Required arguments:
            %   A - State transition matrix (nx x nx)
            %   B - Input matrix (nx x nu)  
            %   Q - State cost matrix (nx x nx, positive semidefinite)
            %   R - Input cost matrix (nu x nu, positive definite)
            %   N - Prediction horizon (positive integer >= 2)
            %
            % Optional name-value pair arguments:
            %   'rho' - ADMM penalty parameter (positive scalar), defaults to 1.0
            %   'fdyn' - Affine dynamics term (nx x 1), defaults to zeros(nx,1)
            %   'verbose' - Verbose output flag (logical), defaults to false
            %   
            %   State constraints (applied to all time steps):
            %   'x_min' - State lower bounds (scalar, nx x 1, or nx x N)
            %   'x_max' - State upper bounds (scalar, nx x 1, or nx x N)
            %   
            %   Input constraints (applied to all time steps):  
            %   'u_min' - Input lower bounds (scalar, nu x 1, or nu x (N-1))
            %   'u_max' - Input upper bounds (scalar, nu x 1, or nu x (N-1))
            %
            %   Solver settings:
            %   'abs_pri_tol' - Solution tolerance for primal variables
            %   'abs_dua_tol' - Solution tolerance for dual variables
            %   'max_iter' - Maximum number of iterations before returning
            %   'check_termination' - Number of iterations to skip before checking termination
            %   'en_state_bound' - Enable or disable bound constraints on state
            %   'en_input_bound' - Enable or disable bound constraints on input
            %   'adaptive_rho' - Enable adaptive rho (logical), defaults to false
            %   'adaptive_rho_min' - Minimum rho value (positive scalar), defaults to 0.1
            %   'adaptive_rho_max' - Maximum rho value (positive scalar), defaults to 10.0
            %   'adaptive_rho_enable_clipping' - Enable rho clipping (logical), defaults to true
            %
            % Example:
            %   solver = TinyMPC();
            %   solver.setup(A, B, Q, R, 20, 'u_min', -5, 'u_max', 5, 'rho', 1.5, 'verbose', true);
            
            if obj.is_setup
                warning('TinyMPC:AlreadySetup', ...
                    'Solver already setup. Create a new instance to re-setup.');
                return;
            end
            
            % Validate required arguments
            if nargin < 6
                error('TinyMPC:InvalidInput', ...
                    'setup() requires exactly 5 arguments: A, B, Q, R, N');
            end
            
            % Validate dimensions
            if ~(isnumeric(N) && isscalar(N) && N >= 2 && N == round(N))
                error('TinyMPC:InvalidInput', 'N must be an integer >= 2, got %g', N);
            end
            
            % Store problem dimensions from matrix sizes
            obj.nx = size(A, 1);
            obj.nu = size(B, 2);
            obj.N = N;
            
            % Store and validate problem matrices
            obj.A = A;
            obj.B = B;
            obj.Q = Q;
            obj.R = R;
            
            % Validate matrix dimensions and properties
            obj.validate_matrices();
            
            try
                % Parse optional arguments with comprehensive validation
                p = inputParser;
                addParameter(p, 'rho', 1.0, @(x) obj.validate_scalar_positive(x, 'rho'));
                addParameter(p, 'fdyn', zeros(obj.nx, 1), @(x) obj.validate_affine_term(x));
                addParameter(p, 'verbose', false, @(x) obj.validate_logical(x, 'verbose'));
                
                % Settings parameters
                addParameter(p, 'abs_pri_tol', 1e-4, @(x) obj.validate_scalar_positive(x, 'abs_pri_tol'));
                addParameter(p, 'abs_dua_tol', 1e-4, @(x) obj.validate_scalar_positive(x, 'abs_dua_tol'));
                addParameter(p, 'max_iter', 100, @(x) obj.validate_scalar_positive_integer(x, 'max_iter'));
                addParameter(p, 'check_termination', 1, @(x) obj.validate_scalar_positive_integer(x, 'check_termination'));
                addParameter(p, 'en_state_bound', false, @(x) obj.validate_logical(x, 'en_state_bound'));
                addParameter(p, 'en_input_bound', false, @(x) obj.validate_logical(x, 'en_input_bound'));
                
                % Constraint parameters
                addParameter(p, 'x_min', [], @(x) obj.validate_bounds(x, obj.nx, obj.N, 'x_min'));
                addParameter(p, 'x_max', [], @(x) obj.validate_bounds(x, obj.nx, obj.N, 'x_max'));
                addParameter(p, 'u_min', [], @(x) obj.validate_bounds(x, obj.nu, obj.N-1, 'u_min'));
                addParameter(p, 'u_max', [], @(x) obj.validate_bounds(x, obj.nu, obj.N-1, 'u_max'));
                
                % Adaptive rho parameters
                addParameter(p, 'adaptive_rho', false, @(x) obj.validate_logical(x, 'adaptive_rho'));
                addParameter(p, 'adaptive_rho_min', 0.1, @(x) obj.validate_scalar_positive(x, 'adaptive_rho_min'));
                addParameter(p, 'adaptive_rho_max', 10.0, @(x) obj.validate_scalar_positive(x, 'adaptive_rho_max'));
                addParameter(p, 'adaptive_rho_enable_clipping', true, @(x) obj.validate_logical(x, 'adaptive_rho_enable_clipping'));
                
                parse(p, varargin{:});
                
                % Extract validated parameters
                f = p.Results.fdyn;
                obj.rho = p.Results.rho;
                verbose = p.Results.verbose;
                
                % Update settings structure
                obj.settings.abs_pri_tol = p.Results.abs_pri_tol;
                obj.settings.abs_dua_tol = p.Results.abs_dua_tol;
                obj.settings.max_iter = p.Results.max_iter;
                obj.settings.check_termination = p.Results.check_termination;
                obj.settings.en_state_bound = logical(p.Results.en_state_bound);
                obj.settings.en_input_bound = logical(p.Results.en_input_bound);
                obj.settings.adaptive_rho = logical(p.Results.adaptive_rho);
                obj.settings.adaptive_rho_min = p.Results.adaptive_rho_min;
                obj.settings.adaptive_rho_max = p.Results.adaptive_rho_max;
                obj.settings.adaptive_rho_enable_clipping = logical(p.Results.adaptive_rho_enable_clipping);
                
                % Process constraint bounds with intelligent defaults
                obj.x_min = obj.process_bounds(p.Results.x_min, obj.nx, obj.N, -1e17, 'x_min');
                obj.x_max = obj.process_bounds(p.Results.x_max, obj.nx, obj.N, +1e17, 'x_max');
                obj.u_min = obj.process_bounds(p.Results.u_min, obj.nu, obj.N-1, -1e17, 'u_min');
                obj.u_max = obj.process_bounds(p.Results.u_max, obj.nu, obj.N-1, +1e17, 'u_max');
                
                % Call MEX setup function (adaptive rho handled in MATLAB layer)
                status = tinympc_matlab('setup', obj.A, obj.B, f, obj.Q, obj.R, ...
                    obj.rho, obj.nx, obj.nu, obj.N, obj.x_min, obj.x_max, obj.u_min, obj.u_max, verbose);
                
                if status == 0
                    obj.is_setup = true;
                    
                    if verbose
                        fprintf('TinyMPC solver setup successful (nx=%d, nu=%d, N=%d)\n', ...
                            obj.nx, obj.nu, obj.N);
                    end
                else
                    error('TinyMPC:SetupFailed', ...
                        'TinyMPC solver setup failed with status: %d', status);
                end
                
            catch ME
                if contains(ME.identifier, 'TinyMPC:')
                    rethrow(ME);  % Re-throw our custom errors as-is
                else
                    error('TinyMPC:SetupError', ...
                        'Error during solver setup: %s', ME.message);
                end
            end
        end
        
        function set_x0(obj, x0)
            % Set the initial state for the MPC problem (Python-compatible name)
            %
            % Arguments:
            %   x0 - Initial state vector (nx x 1 or 1 x nx)
            
            obj.check_setup();
            
            % Validate and reshape input
            x0 = obj.validate_and_reshape_vector(x0, obj.nx, 'initial state x0');
            
            try
                tinympc_matlab('set_x0', x0, false);  % Use false for verbose internally
            catch ME
                error('TinyMPC:SetInitialStateFailed', ...
                    'Failed to set initial state: %s', ME.message);
            end
        end
        
        function set_x_ref(obj, x_ref)
            % Set state reference trajectory (Python-compatible name)
            %
            % Arguments:
            %   x_ref - State reference, can be:
            %           - Single reference state (nx x 1 or 1 x nx) - expanded to full horizon
            %           - Full trajectory (nx x N) - used as-is
            %           - Scalar (expanded to constant reference for all states and time)
            %
            % Examples:
            %   obj.set_x_ref([1; 0; 0; 0]);        % Single reference state
            %   obj.set_x_ref(ref_trajectory);      % Full nx x N trajectory  
            %   obj.set_x_ref(0);                   % Zero reference for all
            
            obj.check_setup();
            
            try
                % Process and validate reference trajectory
                x_ref_processed = obj.process_reference(x_ref, obj.nx, obj.N, 'state reference');
                
                tinympc_matlab('set_x_ref', x_ref_processed, false);
            catch ME
                if contains(ME.identifier, 'TinyMPC:')
                    rethrow(ME);
                else
                    error('TinyMPC:SetStateReferenceFailed', ...
                        'Failed to set state reference: %s', ME.message);
                end
            end
        end
        
        function set_u_ref(obj, u_ref)
            % Set input reference trajectory (Python-compatible name)
            %
            % Arguments:
            %   u_ref - Input reference, can be:
            %           - Single reference input (nu x 1 or 1 x nu) - expanded to full horizon
            %           - Full trajectory (nu x (N-1)) - used as-is
            %           - Scalar (expanded to constant reference for all inputs and time)
            %
            % Examples:
            %   obj.set_u_ref(0);                   % Zero reference for all
            %   obj.set_u_ref([1; 2]);              % Single reference input
            %   obj.set_u_ref(ref_trajectory);      % Full nu x (N-1) trajectory
            
            obj.check_setup();
            
            try
                % Process and validate reference trajectory
                u_ref_processed = obj.process_reference(u_ref, obj.nu, obj.N-1, 'input reference');
                
                tinympc_matlab('set_u_ref', u_ref_processed, false);
            catch ME
                if contains(ME.identifier, 'TinyMPC:')
                    rethrow(ME);
                else
                    error('TinyMPC:SetInputReferenceFailed', ...
                        'Failed to set input reference: %s', ME.message);
                end
            end
        end
        
        function update_settings(obj, varargin)
            % Update solver settings (Python-compatible interface)
            %
            % Usage:
            %   obj.update_settings('abs_pri_tol', 1e-5, 'max_iter', 200)
            %   obj.update_settings('adaptive_rho', true, 'adaptive_rho_min', 0.01)
            %
            % Optional name-value pair arguments:
            %   'abs_pri_tol' - Solution tolerance for primal variables
            %   'abs_dua_tol' - Solution tolerance for dual variables
            %   'max_iter' - Maximum number of iterations before returning
            %   'check_termination' - Number of iterations to skip before checking termination
            %   'en_state_bound' - Enable or disable bound constraints on state
            %   'en_input_bound' - Enable or disable bound constraints on input
            %   'adaptive_rho' - Enable adaptive rho (logical)
            %   'adaptive_rho_min' - Minimum rho value (positive scalar)
            %   'adaptive_rho_max' - Maximum rho value (positive scalar)
            %   'adaptive_rho_enable_clipping' - Enable rho clipping (logical)
            
            obj.check_setup();
            
            % Parse optional arguments
            p = inputParser;
            addParameter(p, 'abs_pri_tol', obj.settings.abs_pri_tol, @(x) obj.validate_scalar_positive(x, 'abs_pri_tol'));
            addParameter(p, 'abs_dua_tol', obj.settings.abs_dua_tol, @(x) obj.validate_scalar_positive(x, 'abs_dua_tol'));
            addParameter(p, 'max_iter', obj.settings.max_iter, @(x) obj.validate_scalar_positive_integer(x, 'max_iter'));
            addParameter(p, 'check_termination', obj.settings.check_termination, @(x) obj.validate_scalar_positive_integer(x, 'check_termination'));
            addParameter(p, 'en_state_bound', obj.settings.en_state_bound, @(x) obj.validate_logical(x, 'en_state_bound'));
            addParameter(p, 'en_input_bound', obj.settings.en_input_bound, @(x) obj.validate_logical(x, 'en_input_bound'));
            addParameter(p, 'adaptive_rho', obj.settings.adaptive_rho, @(x) obj.validate_logical(x, 'adaptive_rho'));
            addParameter(p, 'adaptive_rho_min', obj.settings.adaptive_rho_min, @(x) obj.validate_scalar_positive(x, 'adaptive_rho_min'));
            addParameter(p, 'adaptive_rho_max', obj.settings.adaptive_rho_max, @(x) obj.validate_scalar_positive(x, 'adaptive_rho_max'));
            addParameter(p, 'adaptive_rho_enable_clipping', obj.settings.adaptive_rho_enable_clipping, @(x) obj.validate_logical(x, 'adaptive_rho_enable_clipping'));
            
            parse(p, varargin{:});
            
            % Update settings structure
            obj.settings.abs_pri_tol = p.Results.abs_pri_tol;
            obj.settings.abs_dua_tol = p.Results.abs_dua_tol;
            obj.settings.max_iter = p.Results.max_iter;
            obj.settings.check_termination = p.Results.check_termination;
            obj.settings.en_state_bound = logical(p.Results.en_state_bound);
            obj.settings.en_input_bound = logical(p.Results.en_input_bound);
            obj.settings.adaptive_rho = logical(p.Results.adaptive_rho);
            obj.settings.adaptive_rho_min = p.Results.adaptive_rho_min;
            obj.settings.adaptive_rho_max = p.Results.adaptive_rho_max;
            obj.settings.adaptive_rho_enable_clipping = logical(p.Results.adaptive_rho_enable_clipping);
            
            try
                % Update settings in the C++ solver
                tinympc_matlab('update_settings', obj.settings, false);
            catch ME
                error('TinyMPC:UpdateSettingsFailed', ...
                    'Failed to update settings: %s', ME.message);
            end
        end
        
        function solution = solve(obj)
            % Solve the MPC problem (Python-compatible interface)
            %
            % Returns:
            %   solution - struct with fields:
            %     .states_all - State trajectory (nx x N)
            %     .controls_all - Input trajectory (nu x N-1)
            %     .controls - First control input (nu x 1)
            
            obj.check_setup();
            
            try
                tinympc_matlab('solve', false);
                
                % Get solution from MEX function
                [x_traj, u_traj] = tinympc_matlab('get_solution', false);
                
                % Return solution in Python-compatible format
                solution = struct();
                solution.states_all = x_traj;
                solution.controls_all = u_traj;
                solution.controls = u_traj(:, 1);  % First control input
                
            catch ME
                error('TinyMPC:SolveFailed', ...
                    'Failed to solve MPC problem: %s', ME.message);
            end
        end
        
        function codegen(obj, output_dir)
            % Generate standalone C++ code
            %
            % Arguments:
            %   output_dir - Directory to save generated code
            
            obj.check_setup();
            
            try
                % Call MEX function to generate C++ code
                status = tinympc_matlab('codegen', output_dir, false);
                
                if status ~= 0
                    error('TinyMPC:CodegenFailed', ...
                        'C++ code generation failed with status: %d', status);
                end
                
                % Copy additional build artifacts (Eigen headers, TinyMPC sources, CMakeLists.txt)
                obj.copy_build_artifacts(output_dir);
                
                fprintf('Code generation completed successfully in: %s\n', output_dir);
                
            catch ME
                error('TinyMPC:CodegenFailed', ...
                    'Code generation failed: %s', ME.message);
            end
        end
        
        function codegen_with_sensitivity(obj, output_dir, dK, dP, dC1, dC2)
            % Generate standalone C++ code with sensitivity matrices
            %
            % Arguments:
            %   output_dir - Directory to save generated code
            %   dK - Derivative of feedback gain w.r.t. rho (nu x nx)
            %   dP - Derivative of value function w.r.t. rho (nx x nx)
            %   dC1 - Derivative of first cache matrix w.r.t. rho (nu x nu)
            %   dC2 - Derivative of second cache matrix w.r.t. rho (nx x nx)
            
            obj.check_setup();
            
            % Validate sensitivity matrices
            obj.validate_sensitivity_matrices(dK, dP, dC1, dC2);
            
            % Set sensitivity matrices
            obj.set_sensitivity_matrices(dK, dP, dC1, dC2);
            
            try
                % Call MEX function to generate C++ code with sensitivity matrices
                status = tinympc_matlab('codegen_with_sensitivity', output_dir, dK, dP, dC1, dC2, false);
                
                if status ~= 0
                    error('TinyMPC:CodegenWithSensitivityFailed', ...
                        'C++ code generation with sensitivity matrices failed with status: %d', status);
                end
                
                % Copy additional build artifacts (Eigen headers, TinyMPC sources, CMakeLists.txt)
                obj.copy_build_artifacts(output_dir);
                
                fprintf('Code generation with sensitivity matrices completed successfully in: %s\n', output_dir);
                
            catch ME
                error('TinyMPC:CodegenWithSensitivityFailed', ...
                    'Code generation with sensitivity matrices failed: %s', ME.message);
            end
        end
        
        function set_sensitivity_matrices(obj, dK, dP, dC1, dC2)
            % Set sensitivity matrices for adaptive rho behavior
            %
            % Arguments:
            %   dK - Derivative of feedback gain w.r.t. rho (nu x nx)
            %   dP - Derivative of value function w.r.t. rho (nx x nx)
            %   dC1 - Derivative of first cache matrix w.r.t. rho (nu x nu)
            %   dC2 - Derivative of second cache matrix w.r.t. rho (nx x nx)
            
            obj.check_setup();
            
            % Validate sensitivity matrices
            obj.validate_sensitivity_matrices(dK, dP, dC1, dC2);
            
            % Store sensitivity matrices
            obj.dK = dK;
            obj.dP = dP;
            obj.dC1 = dC1;
            obj.dC2 = dC2;
            
            try
                tinympc_matlab('set_sensitivity_matrices', dK, dP, dC1, dC2, false);
                
                fprintf('Sensitivity matrices set with norms: dK=%.6f, dP=%.6f, dC1=%.6f, dC2=%.6f\n', ...
                    norm(dK), norm(dP), norm(dC1), norm(dC2));
                
            catch ME
                error('TinyMPC:SetSensitivityMatricesFailed', ...
                    'Failed to set sensitivity matrices: %s', ME.message);
            end
        end
        
        function [Kinf, Pinf, Quu_inv, AmBKt] = compute_cache_terms(obj)
            % Compute cache terms for ADMM solver
            %
            % Returns:
            %   Kinf - Infinite horizon feedback gain (nu x nx)
            %   Pinf - Infinite horizon value function (nx x nx)
            %   Quu_inv - Inverse of Quu matrix (nu x nu)
            %   AmBKt - Transpose of (A - B*K) (nx x nx)
            
            obj.check_setup();
            
            try
                % Get current rho value
                current_rho = obj.rho; % Use stored rho value
                
                % Add rho regularization
                Q_rho = obj.Q + current_rho * eye(obj.nx);
                R_rho = obj.R + current_rho * eye(obj.nu);
                
                % Initialize
                Kinf = zeros(obj.nu, obj.nx);
                Pinf = obj.Q;
                
                % Compute infinite horizon solution
                for iter = 1:5000
                    Kinf_prev = Kinf;
                    Kinf = (R_rho + obj.B' * Pinf * obj.B + 1e-8*eye(obj.nu)) \ (obj.B' * Pinf * obj.A);
                    Pinf = Q_rho + obj.A' * Pinf * (obj.A - obj.B * Kinf);
                    
                    if norm(Kinf - Kinf_prev) < 1e-10
                        break;
                    end
                end
                
                AmBKt = (obj.A - obj.B * Kinf)';
                Quu_inv = inv(R_rho + obj.B' * Pinf * obj.B);
                
                % Set cache terms in the solver
                tinympc_matlab('set_cache_terms', Kinf, Pinf, Quu_inv, AmBKt, false);
                
                fprintf('Cache terms computed with norms: Kinf=%.6f, Pinf=%.6f\n', norm(Kinf), norm(Pinf));
                fprintf('C1=%.6f, C2=%.6f\n', norm(Quu_inv), norm(AmBKt));
                
            catch ME
                error('TinyMPC:ComputeCacheTermsFailed', ...
                    'Failed to compute cache terms: %s', ME.message);
            end
        end
        
        function [dK, dP, dC1, dC2] = compute_sensitivity_autograd(obj)
            % Compute sensitivity matrices dK, dP, dC1, dC2 with respect to rho
            % Uses Symbolic Math Toolbox for exact derivatives
            %
            % Returns:
            %   dK - Derivative of feedback gain w.r.t. rho (nu x nx)
            %   dP - Derivative of value function w.r.t. rho (nx x nx)
            %   dC1 - Derivative of first cache matrix w.r.t. rho (nu x nu)
            %   dC2 - Derivative of second cache matrix w.r.t. rho (nx x nx)

            obj.check_setup();

            try
                % Use symbolic differentiation for exact derivatives
                current_rho = obj.rho;
                syms rho_sym real
                Q_rho = obj.Q + rho_sym * eye(obj.nx);
                R_rho = obj.R + rho_sym * eye(obj.nu);

                % Symbolic LQR solution (discrete-time, infinite horizon)
                % Iterative solution for symbolic Pinf and Kinf
                Pinf = Q_rho;
                for iter = 1:1000
                    Kinf = simplify((R_rho + obj.B' * Pinf * obj.B + 1e-8*eye(obj.nu)) \ (obj.B' * Pinf * obj.A));
                    Pinf_next = simplify(Q_rho + obj.A' * Pinf * (obj.A - obj.B * Kinf));
                    if isequaln(Pinf_next, Pinf)
                        break;
                    end
                    Pinf = Pinf_next;
                end

                AmBKt = (obj.A - obj.B * Kinf)';
                Quu_inv = inv(R_rho + obj.B' * Pinf * obj.B);

                % Compute derivatives symbolically
                dK_sym = simplify( diff(Kinf, rho_sym) );
                dP_sym = simplify( diff(Pinf, rho_sym) );
                dC1_sym = simplify( diff(Quu_inv, rho_sym) );
                dC2_sym = simplify( diff(AmBKt, rho_sym) );

                % Substitute current rho value and convert to double
                dK = double(subs(dK_sym, rho_sym, current_rho));
                dP = double(subs(dP_sym, rho_sym, current_rho));
                dC1 = double(subs(dC1_sym, rho_sym, current_rho));
                dC2 = double(subs(dC2_sym, rho_sym, current_rho));

                fprintf('Sensitivity matrices computed via Symbolic Math Toolbox\n');
                
            catch ME
                error('TinyMPC:ComputeSensitivityAutoGradFailed', ...
                    'Failed to compute sensitivity matrices: %s', ME.message);
            end
        end
    end
    
    methods (Access = private)
        function validate_matrices(obj)
            % Validate that all matrices have correct dimensions and properties
            
            % Check dimensions
            if size(obj.A, 1) ~= obj.nx || size(obj.A, 2) ~= obj.nx
                error('TinyMPC:InvalidDimensions', ...
                    'Matrix A must be nx x nx = %d x %d, got %d x %d', ...
                    obj.nx, obj.nx, size(obj.A, 1), size(obj.A, 2));
            end
            
            if size(obj.B, 1) ~= obj.nx || size(obj.B, 2) ~= obj.nu
                error('TinyMPC:InvalidDimensions', ...
                    'Matrix B must be nx x nu = %d x %d, got %d x %d', ...
                    obj.nx, obj.nu, size(obj.B, 1), size(obj.B, 2));
            end
            
            if size(obj.Q, 1) ~= obj.nx || size(obj.Q, 2) ~= obj.nx
                error('TinyMPC:InvalidDimensions', ...
                    'Matrix Q must be nx x nx = %d x %d, got %d x %d', ...
                    obj.nx, obj.nx, size(obj.Q, 1), size(obj.Q, 2));
            end
            
            if size(obj.R, 1) ~= obj.nu || size(obj.R, 2) ~= obj.nu
                error('TinyMPC:InvalidDimensions', ...
                    'Matrix R must be nu x nu = %d x %d, got %d x %d', ...
                    obj.nu, obj.nu, size(obj.R, 1), size(obj.R, 2));
            end
            
            % Check that matrices are real-valued
            if ~isreal(obj.A) || ~isreal(obj.B) || ~isreal(obj.Q) || ~isreal(obj.R)
                error('TinyMPC:InvalidInput', ...
                    'All matrices must be real-valued');
            end
            
            % Check that Q is positive semidefinite
            if ~issymmetric(obj.Q) || min(eig(obj.Q)) < -1e-10
                error('TinyMPC:InvalidInput', ...
                    'Matrix Q must be positive semidefinite');
            end
            
            % Check that R is positive definite
            if ~issymmetric(obj.R) || min(eig(obj.R)) <= 1e-10
                error('TinyMPC:InvalidInput', ...
                    'Matrix R must be positive definite');
            end
        end
        
        function validate_affine_term(obj, f)
            % Validate affine dynamics term
            if ~isempty(f)
                if ~isnumeric(f) || ~isreal(f)
                    error('TinyMPC:InvalidInput', ...
                        'Affine term f must be a real numeric vector');
                end
                
                if ~isequal(size(f), [obj.nx, 1]) && ~isequal(size(f), [1, obj.nx])
                    error('TinyMPC:InvalidDimensions', ...
                        'Affine term f must be nx x 1 = %d x 1 or 1 x %d, got %d x %d', ...
                        obj.nx, obj.nx, size(f, 1), size(f, 2));
                end
            end
        end
        
        function validate_scalar_positive(~, value, name)
            % Validate positive scalar parameter
            if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || value <= 0
                error('TinyMPC:InvalidInput', ...
                    'Parameter %s must be a positive real scalar, got %s', ...
                    name, mat2str(value));
            end
        end
        
        function validate_logical(~, value, name)
            % Validate logical parameter
            if ~islogical(value) && ~(isnumeric(value) && isscalar(value) && (value == 0 || value == 1))
                error('TinyMPC:InvalidInput', ...
                    'Parameter %s must be logical (true/false), got %s', ...
                    name, mat2str(value));
            end
        end
        
        function validate_scalar_positive_integer(~, value, name)
            % Validate positive integer parameter
            if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || value <= 0 || value ~= round(value)
                error('TinyMPC:InvalidInput', ...
                    'Parameter %s must be a positive integer, got %s', ...
                    name, mat2str(value));
            end
        end
        
        function validate_bounds(~, bounds, dim, horizon, name)
            % Validate constraint bounds
            if isempty(bounds)
                return; % Empty bounds are valid (will use default)
            end
            
            if ~isnumeric(bounds) || ~isreal(bounds)
                error('TinyMPC:InvalidInput', ...
                    'Bounds %s must be real numeric, got %s', name, class(bounds));
            end
            
            % Allow scalar, single column, or full matrix
            if isscalar(bounds)
                % Scalar is valid - will be expanded
            elseif isequal(size(bounds), [dim, 1])
                % Single column is valid - will be expanded
            elseif isequal(size(bounds), [1, dim])
                % Single row is valid - will be expanded
            elseif isequal(size(bounds), [dim, horizon])
                % Full matrix is valid
            else
                error('TinyMPC:InvalidDimensions', ...
                    ['Bounds %s must be:\n' ...
                     '  - Scalar (expanded to all elements)\n' ...
                     '  - %d x 1 vector (expanded to all time steps)\n' ...
                     '  - 1 x %d vector (expanded to all time steps)\n' ...
                     '  - %d x %d matrix (full trajectory)\n' ...
                     'Got %d x %d'], ...
                    name, dim, dim, dim, horizon, size(bounds, 1), size(bounds, 2));
            end
        end
        
        function bounds_out = process_bounds(~, bounds, dim, horizon, default_val, name)
            % Process and expand bounds to full matrix
            if isempty(bounds)
                bounds_out = default_val * ones(dim, horizon);
            elseif isscalar(bounds)
                bounds_out = bounds * ones(dim, horizon);
            elseif isequal(size(bounds), [dim, 1])
                bounds_out = repmat(bounds, 1, horizon);
            elseif isequal(size(bounds), [1, dim])
                bounds_out = repmat(bounds', 1, horizon);
            elseif isequal(size(bounds), [dim, horizon])
                bounds_out = bounds;
            else
                error('TinyMPC:InvalidDimensions', ...
                    'Invalid bounds size for %s', name);
            end
        end
        
        function vec_out = validate_and_reshape_vector(~, vec, expected_length, name)
            % Validate and reshape vector to column vector
            if ~isnumeric(vec) || ~isreal(vec)
                error('TinyMPC:InvalidInput', ...
                    '%s must be a real numeric vector, got %s', name, class(vec));
            end
            
            if isscalar(vec)
                vec_out = vec * ones(expected_length, 1);
            elseif isequal(size(vec), [expected_length, 1])
                vec_out = vec;
            elseif isequal(size(vec), [1, expected_length])
                vec_out = vec';
            else
                error('TinyMPC:InvalidDimensions', ...
                    '%s must be %d x 1, 1 x %d, or scalar, got %d x %d', ...
                    name, expected_length, expected_length, size(vec, 1), size(vec, 2));
            end
        end
        
        function ref_out = process_reference(~, ref, dim, horizon, name)
            % Process and validate reference trajectory
            if ~isnumeric(ref) || ~isreal(ref)
                error('TinyMPC:InvalidInput', ...
                    '%s must be real numeric, got %s', name, class(ref));
            end
            
            if isscalar(ref)
                % Scalar reference - expand to full trajectory
                ref_out = ref * ones(dim, horizon);
            elseif isequal(size(ref), [dim, 1])
                % Single reference vector - expand to full trajectory
                ref_out = repmat(ref, 1, horizon);
            elseif isequal(size(ref), [1, dim])
                % Single reference row vector - expand to full trajectory
                ref_out = repmat(ref', 1, horizon);
            elseif isequal(size(ref), [dim, horizon])
                % Full trajectory - use as-is
                ref_out = ref;
            else
                error('TinyMPC:InvalidDimensions', ...
                    ['%s must be:\n' ...
                     '  - Scalar (expanded to constant reference)\n' ...
                     '  - %d x 1 vector (expanded to all time steps)\n' ...
                     '  - 1 x %d vector (expanded to all time steps)\n' ...
                     '  - %d x %d matrix (full trajectory)\n' ...
                     'Got %d x %d'], ...
                    name, dim, dim, dim, horizon, size(ref, 1), size(ref, 2));
            end
        end
        
        function validate_sensitivity_matrices(obj, dK, dP, dC1, dC2)
            % Validate sensitivity matrices dimensions
            if size(dK, 1) ~= obj.nu || size(dK, 2) ~= obj.nx
                error('TinyMPC:InvalidDimensions', ...
                    'dK must be nu x nx = %d x %d, got %d x %d', ...
                    obj.nu, obj.nx, size(dK, 1), size(dK, 2));
            end
            
            if size(dP, 1) ~= obj.nx || size(dP, 2) ~= obj.nx
                error('TinyMPC:InvalidDimensions', ...
                    'dP must be nx x nx = %d x %d, got %d x %d', ...
                    obj.nx, obj.nx, size(dP, 1), size(dP, 2));
            end
            
            if size(dC1, 1) ~= obj.nu || size(dC1, 2) ~= obj.nu
                error('TinyMPC:InvalidDimensions', ...
                    'dC1 must be nu x nu = %d x %d, got %d x %d', ...
                    obj.nu, obj.nu, size(dC1, 1), size(dC1, 2));
            end
            
            if size(dC2, 1) ~= obj.nx || size(dC2, 2) ~= obj.nx
                error('TinyMPC:InvalidDimensions', ...
                    'dC2 must be nx x nx = %d x %d, got %d x %d', ...
                    obj.nx, obj.nx, size(dC2, 1), size(dC2, 2));
            end
            
            % Check that matrices are real-valued
            if ~isreal(dK) || ~isreal(dP) || ~isreal(dC1) || ~isreal(dC2)
                error('TinyMPC:InvalidInput', ...
                    'All sensitivity matrices must be real-valued');
            end
        end
        
        function check_setup(obj)
            % Check if solver is setup, throw error if not
            if ~obj.is_setup
                error('TinyMPC:NotSetup', ...
                    'Solver not setup. Call setup() first before using solve(), set_initial_state(), or set_*_reference() methods.');
            end
        end
        
        function copy_build_artifacts(~, output_dir)
            % Copy additional build artifacts required for standalone compilation
            %
            % Arguments:
            %   output_dir - Directory where generated code is located
            
            try
                % Get the directory of this class file to locate the codegen_src and wrapper directories
                class_path = mfilename('fullpath');
                [wrapper_dir, ~, ~] = fileparts(class_path);
                codegen_src_path = fullfile(wrapper_dir, 'codegen_src');
                wrapper_path = fullfile(wrapper_dir, 'wrapper');
                
                fprintf('Copying additional build artifacts...\n');
                fprintf('Wrapper directory: %s\n', wrapper_dir);
                fprintf('Codegen source path: %s\n', codegen_src_path);
                fprintf('Wrapper path: %s\n', wrapper_path);
                
                % Copy codegen_src contents (include/ and tinympc/ directories)
                if exist(codegen_src_path, 'dir')
                    copyfile(codegen_src_path, output_dir);
                    fprintf('Copied codegen_src directory (Eigen headers and TinyMPC sources)\n');
                else
                    warning('TinyMPC:MissingCodegenSrc', ...
                        'Warning: codegen_src directory not found at: %s', codegen_src_path);
                end
                
                % Copy wrapper files (CMakeLists.txt)
                if exist(wrapper_path, 'dir')
                    wrapper_files = dir(fullfile(wrapper_path, '*'));
                    for i = 1:length(wrapper_files)
                        if ~wrapper_files(i).isdir
                            copyfile(fullfile(wrapper_path, wrapper_files(i).name), ...
                                    fullfile(output_dir, wrapper_files(i).name));
                            fprintf('Copied wrapper file: %s\n', wrapper_files(i).name);
                        end
                    end
                else
                    warning('TinyMPC:MissingWrapper', ...
                        'Warning: wrapper directory not found at: %s', wrapper_path);
                end
                
                % Create empty build directory for convenience
                build_path = fullfile(output_dir, 'build');
                if ~exist(build_path, 'dir')
                    mkdir(build_path);
                    fprintf('Created empty build directory\n');
                end
                
                fprintf('Successfully copied all build artifacts\n');
                
            catch ME
                warning('TinyMPC:CopyArtifactsError', ...
                    'Error copying build artifacts: %s', ME.message);
                % Continue anyway since basic code generation succeeded
            end
        end
    end
end