classdef NmpcControl < handle
    
    properties
        solver
        nx, nu, N
        nlp_x0
        nlp_lbx, nlp_ubx
        nlp_lbg, nlp_ubg
        nlp_p
        
        T_opt
        sol
        idx
        
        % Warmstart
        nlp_lam_x0
        nlp_lam_g0
    end
    
    methods
        function obj = NmpcControl(rocket, tf)
           
            import casadi.*
           % opti = casadi.Opti(); % Optimization problem

            N_segs = ceil(tf/rocket.Ts); % MPC horizon
            nx = 12; % Number of states
            nu = 4;  % Number of inputs
            
            % Decision variables (symbolic)
            N = N_segs + 1; % Index of last point
            X_sym = SX.sym('X_sym', nx, N); % state trajectory
            U_sym = SX.sym('U_sym', nu, N-1); % control trajectory)
            
            % Parameters (symbolic)
            x0_sym  = SX.sym('x0_sym', nx, 1);  % initial state
            ref_sym = SX.sym('ref_sym', 4, 1);  % target position
        
            %X = opti.variable(nx,N);
            %U = opti.variable(nu,N-1);

            %X0 = opti.parameter(nx,1); % parameter variable for initial state
            %ref0 = opti.parameter(4,1); % parameter variable for initial state
            
            % Default state and input constraints
            ubx = inf(nx, 1);
            lbx = -inf(nx, 1);
            ubu = inf(nu, 1);
            lbu = -inf(nu, 1);
      
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
        
            [xs, us] = rocket.trim();
            sys = rocket.linearize(xs, us);
            sys = c2d(sys, rocket.Ts);
            f_discrete = @(x,u) RK4(x,u,rocket.Ts,@rocket.f);
            % Cost
            betaMax = deg2rad(80);
            deltaMax = deg2rad(15);
            PavgMax = 80;
            PavgMin = 50;
            PdiffMax = 20;
            
            Q = diag([20 ... % min angular velocity about x
                      20 ... % min angular velocity about y
                      10 ...  % min angular velocity about z
                      1 ...   % min alpha angle
                      1 ...   % min beta angle
                      190 ... % Track roll reference 
                      45 ...   % min velocity about x
                      45 ...   % min velocity about y
                      45 ...   % min velocity about z
                      400 ... % track x reference
                      400 ... % track y reference
                      400]);  % track z reference
       
            R = diag([15 ...    %delta1
                      15 ...    %delta2
                      0.0001 ... %Pavg
                      0.0001]);  %Pdiff
            
   
            % MPT3 to get terminal sets/constraints
            sysMPT = LTISystem('A', sys.A, 'B', sys.B);
            sysMPT.x.min(5) = -betaMax;
            sysMPT.x.max(5) = betaMax;
            sysMPT.u.min(1) = -deltaMax;
            sysMPT.u.max(1) = deltaMax;
            sysMPT.u.min(2) = -deltaMax;
            sysMPT.u.max(2) = deltaMax;
            sysMPT.u.min(3) = PavgMin-us(3); % take trim into account
            sysMPT.u.max(3) = PavgMax-us(3);
            sysMPT.u.min(4) = -PdiffMax;
            sysMPT.u.max(4) = PdiffMax;

            sysMPT.x.penalty = QuadFunction(Q); 
            sysMPT.u.penalty = QuadFunction(R);
            
            Qf = sysMPT.LQRPenalty.weight;

            x_ref = [0 0 0 0 0 ref_sym(4) 0 0 0 ref_sym(1:3)']';
            u_ref = us;

            cost = 0;
            
            % Equality constraints (Casadi SX), each entry == 0
            eq_constr = [];
            
            % Inequality constraints (Casadi SX), each entry <= 0
            ineq_constr = [];

            eq_constr = cat(1,eq_constr,X_sym(:,1) - x0_sym);
            for i = 1:N-1
                cost = cost + (X_sym(:,i)-x_ref)'*Q*(X_sym(:,i)-x_ref) + (U_sym(:,i)-u_ref)'*R*(U_sym(:,i)-u_ref);
                eq_constr = vertcat(eq_constr,(X_sym(:,i+1) - f_discrete(X_sym(:,i),U_sym(:,i)) ));
            end
            cost = cost + (X_sym(:,N)-x_ref)'*Qf*(X_sym(:,N)-x_ref);

            % For box constraints on state and input, overwrite entries of
            % lbx, ubx, lbu, ubu defined above

            ubx(5) = betaMax;
            lbx(5) = -betaMax;

            ubu(1) = deltaMax;
            lbu(1) = -deltaMax;
            ubu(2) = deltaMax;
            lbu(2) = -deltaMax;
            ubu(3) = PavgMax;
            lbu(3) = PavgMin;
            ubu(4) = PdiffMax;
            lbu(4) = -PdiffMax;
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % ---- Assemble NLP ------
            nlp_x = [X_sym(:); U_sym(:)];
            nlp_p = [x0_sym; ref_sym];
            nlp_f = cost;
            nlp_g = [eq_constr; ineq_constr];
            
            nlp = struct('x', nlp_x, 'p', nlp_p, 'f', nlp_f, 'g', nlp_g);
            
            % ---- Setup solver ------
            opts = struct('ipopt', struct('print_level', 0), 'print_time', false);
            obj.solver = nlpsol('solver', 'ipopt', nlp, opts);
            
            % ---- Assemble NLP bounds ----
            obj.nlp_x0  = zeros(size(nlp_x));
            
            obj.nlp_ubx = [repmat(ubx, N, 1); repmat(ubu, (N-1), 1)];
            obj.nlp_lbx = [repmat(lbx, N, 1); repmat(lbu, (N-1), 1)];
            
            obj.nlp_ubg = [zeros(size(eq_constr)); zeros(size(ineq_constr))];
            obj.nlp_lbg = [zeros(size(eq_constr)); -inf(size(ineq_constr))];
            
            obj.nlp_p = [zeros(size(x0_sym)); zeros(size(ref_sym))];
            
            obj.nlp_lam_x0 = [];
            obj.nlp_lam_g0 = [];
            
            obj.nx = nx;
            obj.nu = nu;
            obj.N = N;
            obj.T_opt = linspace(0, N * rocket.Ts, N);
            
            obj.idx.X = [1, obj.N * obj.nx];
            obj.idx.U = obj.idx.X(2) + [1, (obj.N-1) * obj.nu];
            obj.idx.u0 = obj.idx.U(1) + [0, obj.nu-1];
        end
        
        function [u, T_opt, X_opt, U_opt] = get_u(obj, x0, ref)
            
            obj.solve(x0, ref);
            
            % Evaluate u0
            nlp_x = obj.sol.x;
            id = obj.idx.u0;
            u = full( nlp_x(id(1):id(2)) );      
            
            if nargout > 1, T_opt = obj.get_T_opt(); end
            if nargout > 2, X_opt = obj.get_X_opt(); end
            if nargout > 3, U_opt = obj.get_U_opt(); end
            return
            
            % Additional evaluation
            % Complete trajectory
            % % X_opt = full(reshape(nlp_x(idx_X(1):idx_X(2)), obj.nx, obj.N));
            % % U_opt = full(reshape(nlp_x(idx_U(1):idx_U(2)), obj.nu, obj.N - 1));
            % %
            % % cost_opt = full(sol.f);
            % % constr_opt = full(sol.g);
            % %
            % % stats = obj.solver.stats;
        end
        
        function solve(obj, x0, ref)
            
            % ---- Set the initial state and reference ----
            obj.nlp_p = [x0; ref];     % Initial condition
            obj.nlp_x0(1:obj.nx) = x0; % Initial guess consistent
            
            % ---- Solve the optimization problem ----
            args = {'x0', obj.nlp_x0, ...
                'lbg', obj.nlp_lbg, ...
                'ubg', obj.nlp_ubg, ...
                'lbx', obj.nlp_lbx, ...
                'ubx', obj.nlp_ubx, ...
                'p', obj.nlp_p, ...
                %                 'lam_x0', obj.nlp_lam_x0, ...
                %                 'lam_g0', obj.nlp_lam_g0
                };
            
            obj.sol = obj.solver(args{:});
            if obj.solver.stats.success ~= true
                solve_status_str = obj.solver.stats.return_status;
                fprintf([' [' class(obj) ': ' solve_status_str '] ']);
                obj.sol.x(obj.idx.u0) = nan;
            end
            
            % Use the current solution to speed up the next optimization
            obj.nlp_x0 = obj.sol.x;
            obj.nlp_lam_x0 = obj.sol.lam_x;
            obj.nlp_lam_g0 = obj.sol.lam_g;
        end
        function T_opt = get_T_opt(obj)
            T_opt = obj.T_opt;
        end
        function X_opt = get_X_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.X;
            X_opt = full(reshape(nlp_x(id(1):id(2)), obj.nx, obj.N));
        end
        function U_opt = get_U_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.U;
            U_opt = full(reshape(nlp_x(id(1):id(2)), obj.nu, obj.N - 1));
        end
    end
end

