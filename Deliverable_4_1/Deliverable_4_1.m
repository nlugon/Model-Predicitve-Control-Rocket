%% TODO: This file should produce all the plots for the deliverable
%addpath(fullfile('..', 'src'));
addpath(fullfile('src'));
addpath(fullfile('Deliverable_4_1'));
close all;
clear all;
clc

%% Initialize 
Ts = 1/20; %sampling step
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
%% Compute MPC controllers
H = 4; % Horizon length in seconds

mpc_x = MpcControl_x_4_1(sys_x, Ts, H);
mpc_y = MpcControl_y_4_1(sys_y, Ts, H);
mpc_z = MpcControl_z_4_1(sys_z, Ts, H);
mpc_roll = MpcControl_roll_4_1(sys_roll, Ts, H);
%u = mpc_x.get_u(x, pos_ref)

%% Merge four sub-system controllers into one full-system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% Evaluate once and plot optimal open−loop trajectory,
x0 = zeros(12,1);
ref4 = [2 2 2 deg2rad(40)]';
[u, T_opt, X_opt, U_opt] = mpc.get_u(x0, ref4);
U_opt(:,end+1) = nan; % pad last input to get consistent size with time and state
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4); % Plot as usual

%% Setup reference function
ref = @(t_, x_) ref_EPFL(t_);

% Simulate
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

% Visualize
rocket.anim_rate = 2.5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title

%% Setup reference function for rollmax=50
roll_max = deg2rad(50);
ref_50 = @(t_, x_) ref_EPFL(t_, roll_max);

% Simulate
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref_50);

% Visualize
rocket.anim_rate = 2.5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title