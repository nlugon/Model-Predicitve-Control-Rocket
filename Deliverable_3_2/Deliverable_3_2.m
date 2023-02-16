%% TODO: This file should produce all the plots for the deliverable
%addpath(fullfile('..', 'src'));
addpath(fullfile('src'));
addpath(fullfile('Deliverable_3_2'));
close all;
clear all;
clc

%% Initialize 
Ts = 1/20; %sampling step
Tf = 8; %simulation end time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
%% Compute MPC controllers
H = 4; % Horizon length in seconds
mpc_x = MpcControl_x_3_2(sys_x, Ts, H);
mpc_y = MpcControl_y_3_2(sys_y, Ts, H);
mpc_z = MpcControl_z_3_2(sys_z, Ts, H);
mpc_roll = MpcControl_roll_3_2(sys_roll, Ts, H);

%u = mpc_x.get_u(x, pos_ref)

%% Simulate x in closed loop with ref
x0 = [0; 0; 0; 4]; %(wy, beta, vx, x)
ref_x = -4;
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, ref_x);
ph_x = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);

%% Simulate y in closed loop with ref
y0 = [0; 0; 0; 4]; %(wx, alpha, vy, y)
ref_y = -4;
[T, Y_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, ref_y);
ph_y = rocket.plotvis_sub(T, Y_sub, U_sub, sys_y, xs, us, ref_y);

%% Simulate z in closed loop with ref
z0 = [0; 4]; %(vz, z)
ref_z = -4;
[T, Z_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, ref_z);
ph_z = rocket.plotvis_sub(T, Z_sub, U_sub, sys_z, xs, us, ref_z);

%% Simulate roll in closed loop with ref
roll0 = [0; 0]; %(wz, gamma)
ref_roll = deg2rad(35);
[T, Roll_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, ref_roll);
ph_r = rocket.plotvis_sub(T, Roll_sub, U_sub, sys_roll, xs, us, ref_roll);

%% Evaluate x in open loop with ref
% Evaluate once and plot optimal open−loop trajectory,
x = [0; 0; 0; 4]; %(wy, beta, vx, x)
ref_x = -4;
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x, ref_x);
U_opt(:,end+1) = nan; % pad last input to get consistent size with time and state
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us, ref_x); % Plot as usual

%% Evaluate y in open loop with ref
y = [0; 0; 0; 4]; %(wx, alpha, vy, y)
ref_y = -4;
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(y,ref_y);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);

%% Evaluate z in open loop with ref
z = [0; 4]; %(vz, z)
ref_z  = -4;
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z,ref_z);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

%% Evaluate roll in open loop with ref
roll = [0; pi/4]; %(wz, gamma)
ref_roll  = 35 * pi/180;
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll,ref_roll);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);

%% Evalue X static_state