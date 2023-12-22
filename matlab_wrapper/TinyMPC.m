classdef TinyMPC
    properties
        n = 0; % State dimension
        m = 0; % Input dimension
        N = 0; % Prediction horizon
        A = []; % State transition matrix
        B = []; % Input matrix
        Q = []; % State cost matrix
        R = []; % Input cost matrix
        x_min = []; % State lower bounds
        x_max = []; % State upper bounds
        u_min = []; % Input lower bounds
        u_max = []; % Inpu
        % t upper bounds
        rho = 1.0; % Penalty parameter
        abs_pri_tol = 1e-3; % Absolute primal tolerance
        abs_dual_tol = 1e-3; % Absolute dual tolerance
        max_iter = 1000; % Maximum number of iterations
        check_termination = 1; % Check termination flag
        gen_wrapper = 1;
        lib_name
    end
    
    methods
        function obj = TinyMPC(n, m, N, A, B, Q, R, x_min, x_max, u_min, u_max, rho, abs_pri_tol, abs_dual_tol, max_iter, check_termination)
            % Assign values to properties
            if nargin > 0
                obj.n = n;
                obj.m = m;
                obj.N = N;
                obj.A = A;
                obj.B = B;
                obj.Q = Q;
                obj.R = R;
                obj.x_min = x_min;
                obj.x_max = x_max;
                obj.u_min = u_min;
                obj.u_max = u_max;
                obj.rho = rho;
                obj.abs_pri_tol = abs_pri_tol;
                obj.abs_dual_tol = abs_dual_tol;
                obj.max_iter = max_iter;
                obj.check_termination = check_termination;
                obj.gen_wrapper = 1;
            end
        end
        function up_obj = setup(obj, n, m, N, A, B, Q, R, x_min, x_max, u_min, u_max, varargin)
            % Define default values
            default_rho = 1.0;
            default_abs_pri_tol = 1e-3;
            default_abs_dual_tol = 1e-3;
            default_max_iter = 1000;
            default_check_termination = 1;
            
            parser = inputParser;
            
            % Define input arguments and their default values
            addOptional(parser, 'rho', default_rho);
            addOptional(parser, 'abs_pri_tol', default_abs_pri_tol);
            addOptional(parser, 'abs_dual_tol', default_abs_dual_tol);
            addOptional(parser, 'max_iter', default_max_iter);
            addOptional(parser, 'check_termination', default_check_termination);
            
            % Parse the input arguments
            parse(parser, varargin{:});
            results = parser.Results;
        
            % Update object properties
            obj.n = n;
            obj.m = m;
            obj.N = N;
            obj.A = A;
            obj.B = B;
            obj.Q = Q;
            obj.R = R;
            obj.x_min = x_min;
            obj.x_max = x_max;
            obj.u_min = u_min;
            obj.u_max = u_max;
            obj.rho = results.rho;
            obj.abs_pri_tol = results.abs_pri_tol;
            obj.abs_dual_tol = results.abs_dual_tol;
            obj.max_iter = results.max_iter;
            obj.check_termination = results.check_termination;
        
            up_obj = obj;
        end
        function success = tiny_codegen(obj, tinympc_dir, output_dir)
            % Create pointers for your data (assuming self.A, self.B, etc., are arrays)
            A_ptr = libpointer('doublePtr', obj.A);
            B_ptr = libpointer('doublePtr', obj.B);
            Q_ptr = libpointer('doublePtr', obj.Q);
            R_ptr = libpointer('doublePtr', obj.R);
            x_min_ptr = libpointer('doublePtr', obj.x_min(:));
            x_max_ptr = libpointer('doublePtr', obj.x_max(:));
            u_min_ptr = libpointer('doublePtr', obj.u_min(:));
            u_max_ptr = libpointer('doublePtr', obj.u_max(:));
            tinyMPCDirPtr = libpointer('cstring', tinympc_dir);
            outputDirPtr = libpointer('cstring', output_dir);
            % Call the C function
            success = calllib('libtinympcShared', 'tiny_codegen', ...
                    int32(obj.n), int32(obj.m), int32(obj.N), ...
                    A_ptr, B_ptr, Q_ptr, R_ptr, ...
                    x_min_ptr, x_max_ptr, u_min_ptr, u_max_ptr, ...
                    double(obj.rho), double(obj.abs_pri_tol), double(obj.abs_dual_tol), ...
                    int32(obj.max_iter), int32(obj.check_termination), ...
                    int32(obj.gen_wrapper), tinyMPCDirPtr, outputDirPtr);
        end
        
        function up_obj = load_lib(obj, libDir, varargin)
            try
                % Load the library
                loadlibrary(libDir, varargin{:});
                [~, libName, ~] = fileparts(libDir);
                disp([libName, ' loaded successfully']);
                obj.lib_name = libName;
            catch ME
                rethrow(ME);
            end
            up_obj = obj;
        end
        
        function unload_lib(obj, libName)
            % Unload the specified library
            try
                if libisloaded(libName)
                    unloadlibrary(libName);
                    disp([libName, ' unloaded successfully']);
                else
                    disp([libName, ' is not loaded']);
                end
            catch ME
                disp(['Error unloading ', libName, ': ', ME.message]);
            end
        end

        function compile_lib(obj, dir)
            original_directory = pwd();
            % Specify the path to the build directory
            build_directory = fullfile(dir, 'build');
        
            % Make sure the build directory exists
            if ~exist(build_directory, 'dir')
                mkdir(build_directory);
            end
        
            % Change directory to the build directory
            cd(build_directory);
        
            try
                % Run CMake configuration
                cmake_configure_cmd = {'cmake', dir};
                system(strjoin(cmake_configure_cmd, ' '));
        
                % Run the build process
                cmake_build_cmd = {'cmake', '--build', '.'};
                system(strjoin(cmake_build_cmd, ' '));
            catch ME
                % Restore the original directory if an error occurs
                cd(original_directory);
                rethrow(ME);
            end
        
            % Restore the original directory after successful execution
            cd(original_directory);
        end

        function set_x0(obj, x0, verbose_int)
            x0_ptr = libpointer('singlePtr', x0);
            calllib(obj.lib_name, 'set_x0', x0_ptr, int32(verbose_int));
        end
        
        function solve(obj, verbose_int)
            calllib(obj.lib_name, 'call_tiny_solve', int32(verbose_int));
        end
        
        function u = get_u(obj, verbose_int)
            u_soln = zeros(1, obj.N - 1,'single');
            u_soln_ptr = libpointer('singlePtr', u_soln);
            calllib(obj.lib_name, 'get_u', u_soln_ptr, int32(verbose_int));
            u = get(u_soln_ptr, 'Value');
        end
    
    end
end