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
        
        % Adaptive rho parameters
        adaptive_rho = false; % Enable adaptive rho
        adaptive_rho_min = 0.1; % Minimum rho value
        adaptive_rho_max = 10.0; % Maximum rho value
        adaptive_rho_enable_clipping = true; % Enable rho clipping
        
        % Sensitivity matrices for adaptive rho
        dK = []; % Derivative of feedback gain w.r.t. rho
        dP = []; % Derivative of value function w.r.t. rho
        dC1 = []; % Derivative of first cache matrix w.r.t. rho
        dC2 = []; % Derivative of second cache matrix w.r.t. rho
    end
    
    methods
        function obj = TinyMPC(nx, nu, N, A, B, Q, R)
            % Constructor for TinyMPC class
            % 
            % Usage:
            %   solver = TinyMPC(nx, nu, N, A, B, Q, R)
            %
            % Required arguments:
            %   nx - State dimension (positive integer)
            %   nu - Input dimension (positive integer)
            %   N - Prediction horizon (positive integer >= 2)
            %   A - State transition matrix (nx x nx)
            %   B - Input matrix (nx x nu)
            %   Q - State cost matrix (nx x nx, positive semidefinite)
            %   R - Input cost matrix (nu x nu, positive definite)
            %
            % Notes:
            %   - All matrices should be real-valued
            %   - Use setup() method to configure constraints and solver options
            %   - Call setup() before using solve()
            
            if nargin < 7
                error('TinyMPC:InvalidInput', ...
                    'TinyMPC requires exactly 7 arguments: nx, nu, N, A, B, Q, R');
            end
            
            % Validate dimensions
            if ~(isnumeric(nx) && isscalar(nx) && nx > 0 && nx == round(nx))
                error('TinyMPC:InvalidInput', 'nx must be a positive integer, got %s', class(nx));
            end
            if ~(isnumeric(nu) && isscalar(nu) && nu > 0 && nu == round(nu))
                error('TinyMPC:InvalidInput', 'nu must be a positive integer, got %s', class(nu));
            end
            if ~(isnumeric(N) && isscalar(N) && N >= 2 && N == round(N))
                error('TinyMPC:InvalidInput', 'N must be an integer >= 2, got %g', N);
            end
            
            % Store problem dimensions
            obj.nx = nx;
            obj.nu = nu;
            obj.N = N;
            
            % Store and validate problem matrices
            obj.A = A;
            obj.B = B;
            obj.Q = Q;
            obj.R = R;
            
            % Validate matrix dimensions and properties
            obj.validate_matrices();
        end
        
        function success = setup(obj, varargin)
            % Setup the TinyMPC solver with optional constraints and parameters
            %
            % Usage:
            %   prob.setup()  % Default settings
            %   prob.setup('rho', 1.5, 'verbose', true)
            %   prob.setup('u_min', -2, 'u_max', 2, 'rho', 1.0)
            %   prob.setup('x_min', x_min_array, 'x_max', x_max_array, ...)
            %
            % Optional name-value pair arguments:
            %   'f' - Affine dynamics term (nx x 1), defaults to zeros(nx,1)
            %   'rho' - ADMM penalty parameter (positive scalar), defaults to 1.0
            %   'verbose' - Verbose output flag (logical), defaults to false
            %   'max_iter' - Maximum iterations (positive integer), defaults to 100
            %   
            %   State constraints (applied to all time steps):
            %   'x_min' - State lower bounds (scalar, nx x 1, or nx x N)
            %   'x_max' - State upper bounds (scalar, nx x 1, or nx x N)
            %   
            %   Input constraints (applied to all time steps):  
            %   'u_min' - Input lower bounds (scalar, nu x 1, or nu x (N-1))
            %   'u_max' - Input upper bounds (scalar, nu x 1, or nu x (N-1))
            %
            %   Adaptive rho parameters:
            %   'adaptive_rho' - Enable adaptive rho (logical), defaults to false
            %   'adaptive_rho_min' - Minimum rho value (positive scalar), defaults to 0.1
            %   'adaptive_rho_max' - Maximum rho value (positive scalar), defaults to 10.0
            %   'adaptive_rho_enable_clipping' - Enable rho clipping (logical), defaults to true
            %
            % Returns:
            %   success - true if setup was successful, false otherwise
            %
            % Example:
            %   prob = TinyMPC(4, 1, 20, A, B, Q, R);
            %   prob.setup('u_min', -5, 'u_max', 5, 'rho', 1.5, 'verbose', true);
            
            if obj.is_setup
                warning('TinyMPC:AlreadySetup', ...
                    'Solver already setup. Call reset() first to re-setup.');
                success = false;
                return;
            end
            
            try
                % Parse optional arguments with comprehensive validation
                p = inputParser;
                addParameter(p, 'f', zeros(obj.nx, 1), @(x) obj.validate_affine_term(x));
                addParameter(p, 'rho', 1.0, @(x) obj.validate_scalar_positive(x, 'rho'));
                addParameter(p, 'verbose', false, @(x) obj.validate_logical(x, 'verbose'));
                addParameter(p, 'max_iter', 100, @(x) obj.validate_scalar_positive_integer(x, 'max_iter'));
                
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
                f = p.Results.f;
                rho = p.Results.rho;
                verbose = p.Results.verbose;
                
                % Store adaptive rho parameters
                obj.adaptive_rho = p.Results.adaptive_rho;
                obj.adaptive_rho_min = p.Results.adaptive_rho_min;
                obj.adaptive_rho_max = p.Results.adaptive_rho_max;
                obj.adaptive_rho_enable_clipping = p.Results.adaptive_rho_enable_clipping;
                
                % Process constraint bounds with intelligent defaults
                x_min = obj.process_bounds(p.Results.x_min, obj.nx, obj.N, -1e17, 'x_min');
                x_max = obj.process_bounds(p.Results.x_max, obj.nx, obj.N, +1e17, 'x_max');
                u_min = obj.process_bounds(p.Results.u_min, obj.nu, obj.N-1, -1e17, 'u_min');
                u_max = obj.process_bounds(p.Results.u_max, obj.nu, obj.N-1, +1e17, 'u_max');
                
                % Call MEX setup function with adaptive rho parameters
                status = tinympc_matlab('setup', obj.A, obj.B, f, obj.Q, obj.R, ...
                    rho, obj.nx, obj.nu, obj.N, x_min, x_max, u_min, u_max, verbose, ...
                    obj.adaptive_rho, obj.adaptive_rho_min, obj.adaptive_rho_max, obj.adaptive_rho_enable_clipping);
                
                if status == 0
                    obj.is_setup = true;
                    success = true;
                    
                    if verbose
                        fprintf('TinyMPC solver setup successful (nx=%d, nu=%d, N=%d)\n', ...
                            obj.nx, obj.nu, obj.N);
                    end
                else
                    success = false;
                    error('TinyMPC:SetupFailed', ...
                        'TinyMPC solver setup failed with status: %d', status);
                end
                
            catch ME
                success = false;
                if contains(ME.identifier, 'TinyMPC:')
                    rethrow(ME);  % Re-throw our custom errors as-is
                else
                    error('TinyMPC:SetupError', ...
                        'Error during solver setup: %s', ME.message);
                end
            end
        end
        
        function success = set_initial_state(obj, x0)
            % Set the initial state for the MPC problem
            %
            % Arguments:
            %   x0 - Initial state vector (nx x 1 or 1 x nx)
            %
            % Returns:
            %   success - true if successful, false otherwise
            
            obj.check_setup();
            
            % Validate and reshape input
            x0 = obj.validate_and_reshape_vector(x0, obj.nx, 'initial state x0');
            
            try
                tinympc_matlab('set_x0', x0, false);  % Use false for verbose internally
                success = true;
            catch ME
                success = false;
                error('TinyMPC:SetInitialStateFailed', ...
                    'Failed to set initial state: %s', ME.message);
            end
        end
        
        function success = set_state_reference(obj, x_ref)
            % Set state reference trajectory (flexible input handling)
            %
            % Arguments:
            %   x_ref - State reference, can be:
            %           - Single reference state (nx x 1 or 1 x nx) - expanded to full horizon
            %           - Full trajectory (nx x N) - used as-is
            %           - Scalar (expanded to constant reference for all states and time)
            %
            % Returns:
            %   success - true if successful, false otherwise
            %
            % Examples:
            %   prob.set_state_reference([1; 0; 0; 0]);        % Single reference state
            %   prob.set_state_reference(ref_trajectory);      % Full nx x N trajectory  
            %   prob.set_state_reference(0);                   % Zero reference for all
            
            obj.check_setup();
            
            try
                % Process and validate reference trajectory
                x_ref_processed = obj.process_reference(x_ref, obj.nx, obj.N, 'state reference');
                
                tinympc_matlab('set_x_ref', x_ref_processed, false);
                success = true;
            catch ME
                success = false;
                if contains(ME.identifier, 'TinyMPC:')
                    rethrow(ME);
                else
                    error('TinyMPC:SetStateReferenceFailed', ...
                        'Failed to set state reference: %s', ME.message);
                end
            end
        end
        
        function success = set_input_reference(obj, u_ref)
            % Set input reference trajectory (flexible input handling)
            %
            % Arguments:
            %   u_ref - Input reference, can be:
            %           - Single reference input (nu x 1 or 1 x nu) - expanded to full horizon
            %           - Full trajectory (nu x (N-1)) - used as-is
            %           - Scalar (expanded to constant reference for all inputs and time)
            %
            % Returns:
            %   success - true if successful, false otherwise
            %
            % Examples:
            %   prob.set_input_reference(0);                   % Zero reference for all
            %   prob.set_input_reference([1; 2]);              % Single reference input
            %   prob.set_input_reference(ref_trajectory);      % Full nu x (N-1) trajectory
            
            obj.check_setup();
            
            try
                % Process and validate reference trajectory
                u_ref_processed = obj.process_reference(u_ref, obj.nu, obj.N-1, 'input reference');
                
                tinympc_matlab('set_u_ref', u_ref_processed, false);
                success = true;
            catch ME
                success = false;
                if contains(ME.identifier, 'TinyMPC:')
                    rethrow(ME);
                else
                    error('TinyMPC:SetInputReferenceFailed', ...
                        'Failed to set input reference: %s', ME.message);
                end
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
                error('TinyMPC:InvalidDimensions', ...
                    'x_min must be nx x N = %d x %d, got %d x %d', ...
                    obj.nx, obj.N, size(x_min, 1), size(x_min, 2));
            end
            if size(x_max, 1) ~= obj.nx || size(x_max, 2) ~= obj.N
                error('TinyMPC:InvalidDimensions', ...
                    'x_max must be nx x N = %d x %d, got %d x %d', ...
                    obj.nx, obj.N, size(x_max, 1), size(x_max, 2));
            end
            if size(u_min, 1) ~= obj.nu || size(u_min, 2) ~= (obj.N-1)
                error('TinyMPC:InvalidDimensions', ...
                    'u_min must be nu x (N-1) = %d x %d, got %d x %d', ...
                    obj.nu, obj.N-1, size(u_min, 1), size(u_min, 2));
            end
            if size(u_max, 1) ~= obj.nu || size(u_max, 2) ~= (obj.N-1)
                error('TinyMPC:InvalidDimensions', ...
                    'u_max must be nu x (N-1) = %d x %d, got %d x %d', ...
                    obj.nu, obj.N-1, size(u_max, 1), size(u_max, 2));
            end
            
            try
                tinympc_matlab('set_bounds', x_min(:), x_max(:), u_min(:), u_max(:), false);
                success = true;
            catch ME
                success = false;
                error('TinyMPC:SetBoundsFailed', ...
                    'Failed to set bounds: %s', ME.message);
            end
        end
        
        function success = solve(obj)
            % Solve the MPC problem
            %
            % Returns:
            %   success - true if solve was successful, false otherwise
            
            obj.check_setup();
            
            try
                tinympc_matlab('solve', false);
                success = true;
            catch ME
                success = false;
                error('TinyMPC:SolveFailed', ...
                    'Failed to solve MPC problem: %s', ME.message);
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
                % Get solution from MEX function
                [x_traj, u_traj] = tinympc_matlab('get_solution', false);
                
            catch ME
                error('TinyMPC:GetSolutionFailed', ...
                    'Failed to get solution: %s', ME.message);
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
                status = tinympc_matlab('codegen', output_dir, false);
                success = (status == 0);
                
                if success
                    fprintf('Code generation completed successfully in: %s\n', output_dir);
                end
                
            catch ME
                success = false;
                error('TinyMPC:CodegenFailed', ...
                    'Code generation failed: %s', ME.message);
            end
        end
        
        function success = codegen_with_sensitivity(obj, output_dir, dK, dP, dC1, dC2)
            % Generate standalone C++ code with sensitivity matrices
            %
            % Arguments:
            %   output_dir - Directory to save generated code
            %   dK - Derivative of feedback gain w.r.t. rho (nu x nx)
            %   dP - Derivative of value function w.r.t. rho (nx x nx)
            %   dC1 - Derivative of first cache matrix w.r.t. rho (nu x nu)
            %   dC2 - Derivative of second cache matrix w.r.t. rho (nx x nx)
            %
            % Returns:
            %   success - true if successful, false otherwise
            
            obj.check_setup();
            
            % Validate sensitivity matrices
            obj.validate_sensitivity_matrices(dK, dP, dC1, dC2);
            
            % Set sensitivity matrices
            obj.set_sensitivity_matrices(dK, dP, dC1, dC2);
            
            try
                status = tinympc_matlab('codegen_with_sensitivity', output_dir, dK, dP, dC1, dC2, false);
                success = (status == 0);
                
                if success
                    fprintf('Code generation with sensitivity matrices completed successfully in: %s\n', output_dir);
                end
                
            catch ME
                success = false;
                error('TinyMPC:CodegenWithSensitivityFailed', ...
                    'Code generation with sensitivity matrices failed: %s', ME.message);
            end
        end
        
        function success = set_sensitivity_matrices(obj, dK, dP, dC1, dC2)
            % Set sensitivity matrices for adaptive rho behavior
            %
            % Arguments:
            %   dK - Derivative of feedback gain w.r.t. rho (nu x nx)
            %   dP - Derivative of value function w.r.t. rho (nx x nx)
            %   dC1 - Derivative of first cache matrix w.r.t. rho (nu x nu)
            %   dC2 - Derivative of second cache matrix w.r.t. rho (nx x nx)
            %
            % Returns:
            %   success - true if successful, false otherwise
            
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
                success = true;
                
                fprintf('Sensitivity matrices set with norms: dK=%.6f, dP=%.6f, dC1=%.6f, dC2=%.6f\n', ...
                    norm(dK), norm(dP), norm(dC1), norm(dC2));
                
            catch ME
                success = false;
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
                % Get current rho value (use default if not set)
                rho = 1.0; % Default rho value
                
                % Add rho regularization
                Q_rho = obj.Q + rho * eye(obj.nx);
                R_rho = obj.R + rho * eye(obj.nu);
                
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
            % This is a simplified version - in practice, you would use automatic differentiation
            %
            % Returns:
            %   dK - Derivative of feedback gain w.r.t. rho (nu x nx)
            %   dP - Derivative of value function w.r.t. rho (nx x nx)
            %   dC1 - Derivative of first cache matrix w.r.t. rho (nu x nu)
            %   dC2 - Derivative of second cache matrix w.r.t. rho (nx x nx)
            
            obj.check_setup();
            
            try
                % This is a simplified analytical approximation
                % In practice, you would use automatic differentiation tools
                
                % Get current rho value
                rho = 1.0; % Default rho value
                
                % Compute base solution
                [Kinf, Pinf, Quu_inv, AmBKt] = obj.compute_cache_terms();
                
                % Compute finite difference approximation
                delta_rho = 1e-6;
                
                % Perturb rho and recompute
                Q_rho_pert = obj.Q + (rho + delta_rho) * eye(obj.nx);
                R_rho_pert = obj.R + (rho + delta_rho) * eye(obj.nu);
                
                % Compute perturbed solution
                Kinf_pert = zeros(obj.nu, obj.nx);
                Pinf_pert = obj.Q;
                
                for iter = 1:5000
                    Kinf_prev = Kinf_pert;
                    Kinf_pert = (R_rho_pert + obj.B' * Pinf_pert * obj.B + 1e-8*eye(obj.nu)) \ (obj.B' * Pinf_pert * obj.A);
                    Pinf_pert = Q_rho_pert + obj.A' * Pinf_pert * (obj.A - obj.B * Kinf_pert);
                    
                    if norm(Kinf_pert - Kinf_prev) < 1e-10
                        break;
                    end
                end
                
                AmBKt_pert = (obj.A - obj.B * Kinf_pert)';
                Quu_inv_pert = inv(R_rho_pert + obj.B' * Pinf_pert * obj.B);
                
                % Compute derivatives via finite differences
                dK = (Kinf_pert - Kinf) / delta_rho;
                dP = (Pinf_pert - Pinf) / delta_rho;
                dC1 = (Quu_inv_pert - Quu_inv) / delta_rho;
                dC2 = (AmBKt_pert - AmBKt) / delta_rho;
                
                fprintf('Sensitivity matrices computed via finite differences\n');
                
            catch ME
                error('TinyMPC:ComputeSensitivityAutoGradFailed', ...
                    'Failed to compute sensitivity matrices: %s', ME.message);
            end
        end

        function reset(obj)
            % Reset the solver (cleanup current setup)
            
            if obj.is_setup
                try
                    tinympc_matlab('reset', false);
                catch
                    % Ignore errors during reset
                end
                obj.is_setup = false;
                
                fprintf('TinyMPC solver reset\n');
            end
        end
        
        function delete(obj)
            % Destructor - cleanup when object is destroyed
            obj.reset();
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
    end
end