classdef MpcControl_z_5_1 < MpcControlBase
     properties
         A_bar, B_bar, C_bar % Augmented system for disturbance rejection
         L                   % Estimator gain for disturbance rejection
     end

     methods
         function mpc = MpcControl_z_5_1(sys, Ts, H)
             mpc = mpc@MpcControlBase(sys, Ts, H);

             [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
         end

         % Design a YALMIP optimizer object that takes a steady-state state
         % and input (xs, us) and returns a control input
         function ctrl_opti = setup_controller(mpc, Ts, H)

             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % INPUTS
             %   X(:,1)       - initial state (estimate)
             %   d_est        - disturbance estimate
             %   x_ref, u_ref - reference state/input
             % OUTPUTS
             %   U(:,1)       - input to apply to the system
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

             N_segs = ceil(H/Ts); % Horizon steps
             N = N_segs + 1;      % Last index in 1-based Matlab indexing

             [nx, nu] = size(mpc.B);

             % Targets (Ignore this before Todo 3.3)
             x_ref = sdpvar(nx, 1);
             u_ref = sdpvar(nu, 1);

             % Disturbance estimate (Ignore this before Part 5)
             d_est = sdpvar(1);

             % Predicted state and input trajectories
             X = sdpvar(nx, N);
             U = sdpvar(nu, N-1);

             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

             % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
             %       the DISCRETE-TIME MODEL of your system

             % States: (vz,z) / Input: (Pavg)
             Q = diag([250, 900]);
             R = 0.0009*eye(nu, nu);

             PavgMin = 50-56.667;
             PavgMax = 80-56.667; %take trim into account

             F = [1; -1];
             f = [PavgMax; -PavgMin]; %constraints on U, 50% < Pavg < 80%

            % MPT3 to get terminal sets/constraints
            [~, Qf, ~] = dlqr(mpc.A,mpc.B,Q,R);

            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];

            for i = 1:N-1
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
                con = con + (X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i) + mpc.B*d_est) + (F*U(:,i) <= f);
            end
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref); % keep terminal cost 

             % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

             % Return YALMIP optimizer object
             ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                 {X(:,1), x_ref, u_ref, d_est}, {U(:,1), X, U});
         end


         % Design a YALMIP optimizer object that takes a position reference
         % and returns a feasible steady-state state and input (xs, us)
         function target_opti = setup_steady_state_target(mpc)

             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % INPUTS
             %   ref    - reference to track
             %   d_est  - disturbance estimate
             % OUTPUTS
             %   xs, us - steady-state target
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

             nx = size(mpc.A, 1);

             % Steady-state targets
             xs = sdpvar(nx, 1);
             us = sdpvar;

             % Reference position (Ignore this before Todo 3.3)
             ref = sdpvar;

             % Disturbance estimate (Ignore this before Part 5)
             d_est = sdpvar;

             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
             % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D

             PavgMin = 50-56.667;
             PavgMax = 80-56.667; %take trim into account

             F = [1; -1];
             f = [PavgMax; -PavgMin]; %constraints on U, 50% < Pavg < 80%

             obj = us^2;

             con = [F*us <= f, ...
                    xs == mpc.A*xs + mpc.B*us  + mpc.B*d_est, ...
                    ref == mpc.C*xs];

             % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

             % Compute the steady-state target
             target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
         end


         % Compute augmented system and estimator gain for input disturbance rejection
         function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)

             %%% Design the matrices A_bar, B_bar, L, and C_bar
             %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
             %%% converges to the correct state and constant input disturbance
             %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);

             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
             % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D

             [nx, nu] = size(mpc.B);
             ny = size(mpc.C,1);

             %Check rank of Sigma and observability of (A,C). c.f. Slide 52 of lesson 6
            if (rank([mpc.A-eye(2), mpc.B; mpc.C, 0])) ~= 3 || (rank(obsv(mpc.A, mpc.C)) ~= 2)
                error("Doesn't have full rank or isn't observable");
            end
             
             A_bar = [mpc.A, mpc.B ; zeros(1,nx), 1];
             B_bar = [mpc.B ; zeros(1,nu)];
             C_bar = [mpc.C , zeros(ny,1)];
    
             L = -place(A_bar',C_bar',[0.5,0.6,0.7])';

             % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         end


     end
 end