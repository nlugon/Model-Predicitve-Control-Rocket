classdef NmpcControlOpti < handle
    
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
        function obj = NmpcControlOpti(rocket, tf)
           
            import casadi.*
            opti = casadi.Opti(); % Optimization problem

            N_segs = ceil(tf/rocket.Ts); % MPC horizon
            nx = 12; % Number of states
            nu = 4;  % Number of inputs
            
            % Decision variables (symbolic)
            N = N_segs + 1; % Index of last point
%             X_sym = SX.sym('X_sym', nx, N); % state trajectory
%             U_sym = SX.sym('U_sym', nu, N-1); % control trajectory)
            
            % Parameters (symbolic)
%             x0_sym  = SX.sym('x0_sym', nx, 1);  % initial state
%             ref_sym = SX.sym('ref_sym', 4, 1);  % target position
        
            X_sym = opti.variable(nx,N);
            U_sym = opti.variable(nu,N-1);
            x0_sym = opti.parameter(nx,1);
            ref_sym = opti.parameter(4,1);

        %    X0 = opti.parameter(nx,1); % parameter variable for initial state
         %   ref0 = opti.parameter(4,1); % parameter variable for initial state
            
            % Default state and input constraints
%             ubx = inf(nx, 1);
%             lbx = -inf(nx, 1);
%             ubu = inf(nu, 1);
%             lbu = -inf(nu, 1);


            % ---- objective ---------
         %   opti.minimize(...
         %     -10*sum(X(1,:))  + -10*sum(X(2,:)) + -10*sum(X(3,:)) + ... % Max angular velocity
          %    -10*sum(X(7,:))  + -10*sum(X(8,:)) + -10*sum(X(9,:)) + ... % Max velocity
          %    0.1*U(3,:)*U(3,:)' + ... % Minimize accel
          %    10*U(4,:)*U(4,:)'   + ... % Minimize braking
      

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Cost
            betaMax = deg2rad(80);
            deltaMax = deg2rad(15);
            PavgMax = 80-56.667; %take trim into account
            PavgMin = 50-56.667;
            PdiffMax = 20;

%             Q = diag([500 ... % min angular velocity about x
%                       900 ... % min angular velocity about y
%                       90 ...  % min angular velocity about z
%                       1 ...   % min alpha angle
%                       1 ...   % min beta angle
%                       900 ... % Track roll reference 
%                       1 ...   % min velocity about x
%                       1 ...   % min velocity about y
%                       1 ...   % min velocity about z
%                       900 ... % tra x reference
%                       900 ... % tra y reference
%                       900]);  % tra z reference
            Q = eye(nx);
            R = diag([1,1,5,1]);
            
           
            [xs, us] = rocket.trim();
            sys = rocket.linearize(xs, us);
           
            f_discrete = @(x,u) RK4(x,u,rocket.Ts,@rocket.f);
            % MPT3 to get terminal sets/constraints
            sysMPT = LTISystem('A', sys.A, 'B', sys.B);
            sysMPT.x.min(5) = -betaMax;
            sysMPT.x.max(5) = betaMax;
            sysMPT.u.min(1) = -deltaMax;
            sysMPT.u.max(1) = deltaMax;
            sysMPT.u.min(2) = -deltaMax;
            sysMPT.u.max(2) = deltaMax;
            sysMPT.u.min(3) = PavgMin;
            sysMPT.u.max(3) = PavgMax;
            sysMPT.u.min(4) = -PdiffMax;
            sysMPT.u.max(4) = PdiffMax;

            sysMPT.x.penalty = QuadFunction(Q); 
            sysMPT.u.penalty = QuadFunction(R);
            
            Qf = sysMPT.LQRPenalty.weight;
            Xf = sysMPT.LQRSet;

            x_ref = [0 0 0 0 0 ref_sym(4) 0 0 0 ref_sym(1:3)']';
            u_ref = us;

%             umax = deg2rad(15); %max command on delta2
%             M = [0 0 0   0 1 0   0 0 0   0 0 0; 
%                  0 0 0   0 -1 0   0 0 0   0 0 0]; %constraints on X, abs(beta) <= 7 deg
%             m = [betaMax; betaMax];
%             F = [0 0 0 0;
%                   0 0 0 0]; %constraints on U, abs(delta2) <= 15 deg
%             f = [umax; umax];
            opti.subject_to(X_sym(:,1)==x0_sym);
            opti.subject_to(-deg2rad(85) <= X_sym(5,:) <= deg2rad(85)); % |Beta| <= 85 deg
            opti.subject_to(-deg2rad(15) <= U_sym(1,:) <= deg2rad(15));
            opti.subject_to(-deg2rad(15) <= U_sym(2,:) <= deg2rad(15));
            opti.subject_to(50 <= U_sym(3,:) <= 80);
            opti.subject_to(-20 <= U_sym(4,:) <= 20);
            for i = 1:N-1
                cost = cost + (X_sym(:,i)-x_ref)'*Q*(X_sym(:,i)-x_ref) + (U_sym(:,i)-u_ref)'*R*(U_sym(:,i)-u_ref);
                opti.subject_to(X_sym(:,i+1)== f_discrete(X_sym(:,i),U_sym(:,i)))

            end
            
            cost = cost + (X_sym(:,N)-x_ref)'*Qf*(X_sym(:,N)-x_ref);
            opti.minimize(cost)

%             eq_constr = cat(1,eq_constr,X_sym(:,1) - x0_sym);
%             for i = 1:N-1
%                 cost = cost + (X_sym(:,i)-x_ref)'*Q*(X_sym(:,i)-x_ref) + (U_sym(:,i)-u_ref)'*R*(U_sym(:,i)-u_ref);
%                 eq_constr = cat(1, eq_constr,(X_sym(:,i+1) - f_discrete(X_sym(:,i),U_sym(:,i)) ));
%                 ineq_constr(end+1,1) = (M*X_sym(:,i) - m);
%                 ineq_constr(end+1,1) =  (F*U_sym(:,i) - f);
%             end
%             
%             cost = cost + (X_sym(:,N)-x_ref)'*Qf*(X_sym(:,N)-x_ref);
%             ineq_constr = cat(1,ineq_constr,(Xf.A*X_sym(:,N) - Xf.b)); %terminal constraints+objective
% 
%             For box constraints on state and input, overwrite entries of
%             lbx, ubx, lbu, ubu defined above
% 
%             ubx(5) = betaMax;
%             lbx(5) = -betaMax;
% 
%             ubu(1) = deltaMax;
%             lbu(1) = -deltaMax;
%             ubu(2) = deltaMax;
%             lbu(2) = -deltaMax;
%             ubu(3) = PavgMax;
%             lbu(3) = PavgMin;
%             ubu(4) = PdiffMax;
%             lbu(4) = -PdiffMax;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
       % ---- Setup solver ------
        ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
        opti.solver('ipopt', ops);
        
        % Create function to solve and evaluate opti
        opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
        end
        
        function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)
        
        % ---- Set the initial state and reference ----
        opti.set_value(x0_sym, x0);
        opti.set_value(ref_sym, ref);
        
        % ---- Solve the optimization problem ----
        sol = opti.solve();
        assert(sol.stats.success == 1, 'Error computing optimal input');
        
        u = opti.value(U_sym(:,1));
        
        % Use the current solution to speed up the next optimization
        opti.set_initial(sol.value_variables());
        opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
        end

