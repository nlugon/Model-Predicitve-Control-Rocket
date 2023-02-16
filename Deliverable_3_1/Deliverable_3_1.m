%% TODO: This file should produce all the plots for the deliverable
%addpath(fullfile('..', 'src'));
addpath(fullfile('src'));
addpath(fullfile('Deliverable_3_1'));
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

% Design MPC controller
H = 4; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

%% Simulate x in closed loop
x0 = [0; 0; 0; 4]; %(wy, beta, vx, x)
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph_x = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% Simulate y in closed loop
y0 = [0; 0; 0; 4]; %(wx, alpha, vy, y)
[T, Y_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, 0);
ph_y = rocket.plotvis_sub(T, Y_sub, U_sub, sys_y, xs, us);

%% Simulate z in closed loop
z0 = [0; 4]; %(vz, z)
[T, Z_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, 0);
ph_z = rocket.plotvis_sub(T, Z_sub, U_sub, sys_z, xs, us);

%% Simulate roll in closed loop
roll0 = [0; pi/4]; %(wz, gamma)
[T, Roll_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
ph_r = rocket.plotvis_sub(T, Roll_sub, U_sub, sys_roll, xs, us);

%% Evaluate x in open loop
% Evaluate once and plot optimal open−loop trajectory,
x = [0; 0; 0; 4]; %(wy, beta, vx, x)
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x);
U_opt(:,end+1) = nan; % pad last input to get consistent size with time and state
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual

%% Evaluate y in open loop
y = [0; 0; 0; 4]; %(wx, alpha, vy, y)
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(y);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);

%% Evaluate z in open loop
z = [0; 4]; %(vz, z)
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

%% Evaluate roll in open loop
roll = [0; pi/4]; %(wz, gamma)
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll);
U_opt(:,end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);