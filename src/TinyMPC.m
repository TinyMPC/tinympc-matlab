classdef TinyMPC < handle
    % TinyMPC - MATLAB interface for TinyMPC solver
    
    properties
        % Problem dimensions
        nx = 0; nu = 0; N = 0;
        
        % Problem matrices
        A = []; B = []; Q = []; R = [];
        rho = 1.0;
        
        % Solver state
        is_setup = false;
        settings = struct();
        
        % Constraint bounds
        x_min = []; x_max = []; u_min = []; u_max = [];
        
        % Sensitivity matrices
        dK = []; dP = []; dC1 = []; dC2 = [];
    end
    
    methods
        function obj = TinyMPC()
            % Constructor - initialize default settings
            obj.settings.abs_pri_tol = 1e-4;
            obj.settings.abs_dua_tol = 1e-4;
            obj.settings.max_iter = 100;
            obj.settings.check_termination = 1;
            obj.settings.en_state_bound = false;
            obj.settings.en_input_bound = false;
            obj.settings.adaptive_rho = false;
            obj.settings.adaptive_rho_min = 0.1;
            obj.settings.adaptive_rho_max = 10.0;
            obj.settings.adaptive_rho_enable_clipping = true;
        end
        
        function setup(obj, A, B, Q, R, N, varargin)
            % Setup the TinyMPC solver
            % Usage: obj.setup(A, B, Q, R, N, 'rho', 1.5, 'verbose', true)
            
            % Basic dimension validation
            assert(size(A,1) == size(A,2), 'A must be square');
            assert(size(A,1) == size(B,1), 'A and B row dimensions must match');
            assert(size(Q,1) == size(A,1), 'Q must match A dimensions');
            assert(size(R,1) == size(B,2), 'R must match B column dimension');
            assert(N >= 2, 'N must be >= 2');
            
            % Store dimensions and matrices
            obj.nx = size(A,1); obj.nu = size(B,2); obj.N = N;
            obj.A = A; obj.B = B; obj.Q = Q; obj.R = R;
            
            % Parse options using name-value pairs
            opts = obj.parse_options(struct('rho', 1.0, 'fdyn', [], 'verbose', false, ...
                'x_min', [], 'x_max', [], 'u_min', [], 'u_max', [], ...
                'abs_pri_tol', 1e-4, 'abs_dua_tol', 1e-4, 'max_iter', 100, ...
                'check_termination', 1, 'en_state_bound', false, 'en_input_bound', false, ...
                'adaptive_rho', false, 'adaptive_rho_min', 0.1, 'adaptive_rho_max', 10.0, ...
                'adaptive_rho_enable_clipping', true), varargin{:});
            
            obj.rho = opts.rho;
            
            % Update settings
            obj.settings.abs_pri_tol = opts.abs_pri_tol;
            obj.settings.abs_dua_tol = opts.abs_dua_tol;
            obj.settings.max_iter = opts.max_iter;
            obj.settings.check_termination = opts.check_termination;
            obj.settings.en_state_bound = opts.en_state_bound;
            obj.settings.en_input_bound = opts.en_input_bound;
            obj.settings.adaptive_rho = opts.adaptive_rho;
            obj.settings.adaptive_rho_min = opts.adaptive_rho_min;
            obj.settings.adaptive_rho_max = opts.adaptive_rho_max;
            obj.settings.adaptive_rho_enable_clipping = opts.adaptive_rho_enable_clipping;
            
            % Handle fdyn default
            if isempty(opts.fdyn)
                fdyn = zeros(obj.nx, 1);
            else
                fdyn = opts.fdyn;
            end
            
            % Process bounds with simple expansion
            obj.x_min = obj.expand_bounds(opts.x_min, obj.nx, obj.N, -1e17);
            obj.x_max = obj.expand_bounds(opts.x_max, obj.nx, obj.N, +1e17);
            obj.u_min = obj.expand_bounds(opts.u_min, obj.nu, obj.N-1, -1e17);
            obj.u_max = obj.expand_bounds(opts.u_max, obj.nu, obj.N-1, +1e17);
            
            % Call MEX setup function
            status = tinympc_matlab('setup', obj.A, obj.B, fdyn, obj.Q, obj.R, ...
                obj.rho, obj.nx, obj.nu, obj.N, obj.x_min, obj.x_max, obj.u_min, obj.u_max, opts.verbose);
            
            if status == 0
                obj.is_setup = true;
                
                % Push settings to C++ layer (including adaptive_rho settings)
                tinympc_matlab('update_settings', obj.settings.abs_pri_tol, obj.settings.abs_dua_tol, ...
                    obj.settings.max_iter, obj.settings.check_termination, obj.settings.en_state_bound, ...
                    obj.settings.en_input_bound, obj.settings.adaptive_rho, obj.settings.adaptive_rho_min, ...
                    obj.settings.adaptive_rho_max, obj.settings.adaptive_rho_enable_clipping, false);
                
                if opts.verbose, fprintf('TinyMPC solver setup successful (nx=%d, nu=%d, N=%d)\n', obj.nx, obj.nu, obj.N); end
            else
                error('TinyMPC:SetupFailed', 'Setup failed with status %d', status);
            end
        end
        
        function set_x0(obj, x0)
            % Set initial state
            obj.check_setup();
            tinympc_matlab('set_x0', x0(:), false);
        end
        
        function set_initial_state(obj, x0)
            % Alias for set_x0 (compatibility)
            obj.set_x0(x0);
        end
        
        function set_x_ref(obj, x_ref)
            % Set state reference trajectory
            obj.check_setup();
            x_ref_expanded = obj.expand_matrix(x_ref, obj.nx, obj.N);
            tinympc_matlab('set_x_ref', x_ref_expanded, false);
        end
        
        function set_state_reference(obj, x_ref)
            % Alias for set_x_ref (compatibility)
            obj.set_x_ref(x_ref);
        end
        
        function set_u_ref(obj, u_ref)
            % Set input reference trajectory
            obj.check_setup();
            u_ref_expanded = obj.expand_matrix(u_ref, obj.nu, obj.N-1);
            tinympc_matlab('set_u_ref', u_ref_expanded, false);
        end
        
        function set_input_reference(obj, u_ref)
            % Alias for set_u_ref (compatibility)
            obj.set_u_ref(u_ref);
        end
        
        function update_settings(obj, varargin)
            % Update solver settings
            % Usage: obj.update_settings('max_iter', 200, 'abs_pri_tol', 1e-5)
            obj.check_setup();
            for i = 1:2:length(varargin)
                if isfield(obj.settings, varargin{i})
                    obj.settings.(varargin{i}) = varargin{i+1};
                end
            end
            tinympc_matlab('update_settings', obj.settings.abs_pri_tol, obj.settings.abs_dua_tol, ...
                obj.settings.max_iter, obj.settings.check_termination, obj.settings.en_state_bound, ...
                obj.settings.en_input_bound, obj.settings.adaptive_rho, obj.settings.adaptive_rho_min, ...
                obj.settings.adaptive_rho_max, obj.settings.adaptive_rho_enable_clipping, false);
        end
        
        function status = solve(obj)
            % Solve the MPC problem
            % Returns status code: 0 = success
            obj.check_setup();
            tinympc_matlab('solve', false);
            status = 0; % Always return 0 for success (matches Julia API)
        end
        
        function solution = get_solution(obj)
            % Get current solution
            obj.check_setup();
            [x_traj, u_traj] = tinympc_matlab('get_solution', false);
            
            solution = struct();
            solution.states = x_traj;     % Full state trajectory (nx × N matrix)
            solution.controls = u_traj;   % Optimal control sequence (nu × (N-1) matrix)
        end
        
        function codegen(obj, output_dir)
            % Generate standalone C++ code
            obj.check_setup();
            status = tinympc_matlab('codegen', output_dir, false);
            if status ~= 0
                error('TinyMPC:CodegenFailed', 'Code generation failed with status: %d', status);
            end
            obj.copy_build_artifacts(output_dir);
            fprintf('Code generation completed successfully in: %s\n', output_dir);
        end
        
        function codegen_with_sensitivity(obj, output_dir, dK, dP, dC1, dC2)
            % Generate standalone C++ code with sensitivity matrices
            obj.check_setup();
            obj.validate_sensitivity_matrices(dK, dP, dC1, dC2);
            obj.set_sensitivity_matrices(dK, dP, dC1, dC2);
            
            status = tinympc_matlab('codegen_with_sensitivity', output_dir, dK, dP, dC1, dC2, false);
            if status ~= 0
                error('TinyMPC:CodegenWithSensitivityFailed', 'Code generation with sensitivity failed with status: %d', status);
            end
            obj.copy_build_artifacts(output_dir);
            fprintf('Code generation with sensitivity matrices completed successfully in: %s\n', output_dir);
        end
        
        function set_sensitivity_matrices(obj, dK, dP, dC1, dC2)
            % Set sensitivity matrices for adaptive rho
            obj.check_setup();
            obj.validate_sensitivity_matrices(dK, dP, dC1, dC2);
            obj.dK = dK; obj.dP = dP; obj.dC1 = dC1; obj.dC2 = dC2;
            tinympc_matlab('set_sensitivity_matrices', dK, dP, dC1, dC2, false);
            fprintf('Sensitivity matrices set with norms: dK=%.6f, dP=%.6f, dC1=%.6f, dC2=%.6f\n', ...
                norm(dK), norm(dP), norm(dC1), norm(dC2));
        end
        
        function [Kinf, Pinf, Quu_inv, AmBKt] = compute_cache_terms(obj)
            % Compute cache terms for ADMM solver
            obj.check_setup();
            
            % Add rho regularization
            Q_rho = obj.Q + obj.rho * eye(obj.nx);
            R_rho = obj.R + obj.rho * eye(obj.nu);
            
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
            
            fprintf('Cache terms computed with norms: Kinf=%.6f, Pinf=%.6f\n', norm(Kinf), norm(Pinf));
            fprintf('C1=%.6f, C2=%.6f\n', norm(Quu_inv), norm(AmBKt));
        end
        
        function [dK, dP, dC1, dC2] = compute_sensitivity_autograd(obj)
            % NOTE: THIS IS NUMERICAL DIFFERENTIATION, NOT SYMBOLIC DIFFERENTIATION
            obj.check_setup();
            
            % Use finite differences - this is robust and fast for all system sizes
            h = 1e-6; % Step size for numerical differentiation
            
            % Compute LQR matrices at current rho
            [K0, P0, C1_0, C2_0] = obj.solve_lqr(obj.rho);
            
            % Compute LQR matrices at rho + h
            [K1, P1, C1_1, C2_1] = obj.solve_lqr(obj.rho + h);
            
            % Compute derivatives using finite differences: d/drho ≈ (f(rho+h) - f(rho)) / h
            dK = (K1 - K0) / h;
            dP = (P1 - P0) / h;
            dC1 = (C1_1 - C1_0) / h;
            dC2 = (C2_1 - C2_0) / h;
        end
        
        function set_linear_constraints(obj, Alin_x, blin_x, Alin_u, blin_u)
            % Set linear constraints: Alin_x * x <= blin_x, Alin_u * u <= blin_u
            % 
            % Args:
            %   Alin_x (matrix): State constraint matrix (num_state_constraints x nx)
            %   blin_x (vector): State constraint bounds (num_state_constraints x 1)
            %   Alin_u (matrix): Input constraint matrix (num_input_constraints x nu) 
            %   blin_u (vector): Input constraint bounds (num_input_constraints x 1)
            obj.check_setup();
            
            % Validate dimensions
            if ~isempty(Alin_x)
                assert(size(Alin_x, 2) == obj.nx, 'Alin_x must have nx columns');
                assert(size(blin_x, 1) == size(Alin_x, 1), 'blin_x must match Alin_x rows');
                assert(size(blin_x, 2) == 1, 'blin_x must be a column vector');
            else
                Alin_x = zeros(0, obj.nx);
                blin_x = zeros(0, 1);
            end
            
            if ~isempty(Alin_u)
                assert(size(Alin_u, 2) == obj.nu, 'Alin_u must have nu columns');
                assert(size(blin_u, 1) == size(Alin_u, 1), 'blin_u must match Alin_u rows');
                assert(size(blin_u, 2) == 1, 'blin_u must be a column vector');
            else
                Alin_u = zeros(0, obj.nu);
                blin_u = zeros(0, 1);
            end
            
            % Call MEX function
            tinympc_matlab('set_linear_constraints', Alin_x, blin_x, Alin_u, blin_u, false);
            fprintf('Linear constraints set: %d state constraints, %d input constraints\n', ...
                size(Alin_x, 1), size(Alin_u, 1));
        end
        
        function set_cone_constraints(obj, Acu, qcu, cu, Acx, qcx, cx)
            % Set second-order cone constraints (inputs first, then states)
            %
            % Args:
            %   Acx (vector): Start indices for each state cone constraint (integer vector)
            %   qcx (vector): Dimension of each state cone constraint (integer vector)
            %   cx (vector): Cone parameter for each state cone constraint
            %   Acu (vector): Start indices for each input cone constraint (integer vector)
            %   qcu (vector): Dimension of each input cone constraint (integer vector)
            %   cu (vector): Cone parameter for each input cone constraint
            obj.check_setup();
            
            % Validate dimensions
            if ~isempty(Acu)
                assert(length(qcu) == length(Acu), 'qcu must match Acu length');
                assert(length(cu) == length(Acu), 'cu must match Acu length');
            else
                Acu = [];
                qcu = [];
                cu = [];
            end
            
            if ~isempty(Acx)
                assert(length(qcx) == length(Acx), 'qcx must match Acx length');
                assert(length(cx) == length(Acx), 'cx must match Acx length');
            else
                Acx = [];
                qcx = [];
                cx = [];
            end
            
            % Ensure column vectors
            Acu = Acu(:); qcu = qcu(:); cu = cu(:);
            Acx = Acx(:); qcx = qcx(:); cx = cx(:);
            
            % Call MEX function
            tinympc_matlab('set_cone_constraints', Acx, qcx, cx, Acu, qcu, cu, false); % MEX expects Acx first by historical order
            fprintf('Cone constraints set: %d input cones, %d state cones\n', ...
                length(Acu), length(Acx));
        end
        
        function set_equality_constraints(obj, Aeq_x, beq_x, Aeq_u, beq_u)
            % Set equality constraints: Aeq_x * x == beq_x, Aeq_u * u == beq_u
            % Implemented as two inequalities: <= and >=
            %
            % Args:
            %   Aeq_x (matrix): State equality constraint matrix (num_eq_state x nx)
            %   beq_x (vector): State equality constraint values (num_eq_state x 1)
            %   Aeq_u (matrix): Input equality constraint matrix (num_eq_input x nu)
            %   beq_u (vector): Input equality constraint values (num_eq_input x 1)
            obj.check_setup();
            
            % Handle empty inputs
            if isempty(Aeq_x)
                Aeq_x = zeros(0, obj.nx);
                beq_x = zeros(0, 1);
            end
            if isempty(Aeq_u)
                Aeq_u = zeros(0, obj.nu);
                beq_u = zeros(0, 1);
            end
            
            % Create inequality constraints: Ax <= b and -Ax <= -b (equivalent to Ax >= b)
            % This gives us Ax == b
            Alin_x = [Aeq_x; -Aeq_x];
            blin_x = [beq_x; -beq_x];
            
            Alin_u = [Aeq_u; -Aeq_u];
            blin_u = [beq_u; -beq_u];
            
            % Set the linear constraints
            obj.set_linear_constraints(Alin_x, blin_x, Alin_u, blin_u);
            fprintf('Equality constraints set: %d state equalities, %d input equalities\n', ...
                size(Aeq_x, 1), size(Aeq_u, 1));
        end
        
        function reset(obj)
            % Reset solver
            if obj.is_setup
                tinympc_matlab('reset', false);
                obj.is_setup = false;
            end
        end
    end
    
    methods (Access = private)
        function check_setup(obj)
            % Check if solver is setup
            if ~obj.is_setup
                error('TinyMPC:NotSetup', 'Solver not setup. Call setup() first.');
            end
        end
        
        function [K, P, C1, C2] = solve_lqr(obj, rho_val)
            % Solve LQR problem for given rho value and return all matrices
            % This is the core computation that we differentiate numerically
            
            % Add rho regularization to cost matrices
            Q_rho = obj.Q + rho_val * eye(obj.nx);
            R_rho = obj.R + rho_val * eye(obj.nu);
            
            % Solve discrete-time algebraic Riccati equation
            try
                % Use MATLAB's built-in solver (fastest and most accurate)
                [P, K] = idare(obj.A, obj.B, Q_rho, R_rho);
                K = -K; % idare returns negative gain, we want positive
            catch
                % Fallback to iterative method if idare fails
                P = Q_rho;
                K = zeros(obj.nu, obj.nx);
                for iter = 1:5000
                    K_prev = K;
                    K = (R_rho + obj.B' * P * obj.B + 1e-8*eye(obj.nu)) \ (obj.B' * P * obj.A);
                    P = Q_rho + obj.A' * P * (obj.A - obj.B * K);
                    if iter > 1 && norm(K - K_prev) < 1e-10
                        break;
                    end
                end
            end
            
            % Compute cache matrices that TinyMPC needs
            C1 = inv(R_rho + obj.B' * P * obj.B);  % Quu_inv matrix
            C2 = (obj.A - obj.B * K)';             % AmBKt matrix
        end
        
        function opts = parse_options(~, defaults, varargin)
            % Simple name-value pair parser
            opts = defaults;
            for i = 1:2:length(varargin)
                if i+1 <= length(varargin) && isfield(opts, varargin{i})
                    opts.(varargin{i}) = varargin{i+1};
                end
            end
        end
        
        function bounds = expand_bounds(~, input, dim, horizon, default_val)
            % Simple bounds expansion
            if isempty(input)
                bounds = default_val * ones(dim, horizon);
            elseif isscalar(input)
                bounds = input * ones(dim, horizon);
            elseif isequal(size(input), [dim, 1])
                bounds = repmat(input, 1, horizon);
            elseif isequal(size(input), [1, dim])
                bounds = repmat(input', 1, horizon);
            else
                bounds = input; % Assume user provides correct dimensions
            end
        end
        
        function ref_out = expand_matrix(~, ref, dim, horizon)
            % Simple matrix expansion for references
            if isscalar(ref)
                ref_out = ref * ones(dim, horizon);
            elseif isequal(size(ref), [dim, 1])
                % Single reference vector - expand to full trajectory
                ref_out = repmat(ref, 1, horizon);
            elseif isequal(size(ref), [1, dim])
                ref_out = repmat(ref', 1, horizon);
            else
                ref_out = ref; % Assume correct dimensions
            end
        end
        
        function validate_sensitivity_matrices(obj, dK, dP, dC1, dC2)
            % Basic dimension validation for sensitivity matrices
            assert(isequal(size(dK), [obj.nu, obj.nx]), 'dK must be nu x nx');
            assert(isequal(size(dP), [obj.nx, obj.nx]), 'dP must be nx x nx');
            assert(isequal(size(dC1), [obj.nu, obj.nu]), 'dC1 must be nu x nu');
            assert(isequal(size(dC2), [obj.nx, obj.nx]), 'dC2 must be nx x nx');
        end
        
        function copy_build_artifacts(~, output_dir)
            % Copy additional build artifacts
            try
                class_path = mfilename('fullpath');
                [class_dir, ~, ~] = fileparts(class_path);
                codegen_src_path = fullfile(class_dir, 'codegen_src');
                
                if exist(codegen_src_path, 'dir')
                    copyfile(codegen_src_path, output_dir);
                    fprintf('Copied all contents from codegen_src to output directory\n');
                end
                
                build_path = fullfile(output_dir, 'build');
                if ~exist(build_path, 'dir')
                    mkdir(build_path);
                end
            catch ME
                warning('TinyMPC:CopyArtifactsError', 'Error copying build artifacts: %s', ME.message);
            end
        end
    end
end