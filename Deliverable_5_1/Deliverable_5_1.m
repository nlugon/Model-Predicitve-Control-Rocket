%% TODO: This file should produce all the plots for the deliverable
addpath(fullfile('src'));
addpath(fullfile('Deliverable_5_1'));
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
mpc_x = MpcControl_x_5_1(sys_x, Ts, H);
mpc_y = MpcControl_y_5_1(sys_y, Ts, H);
mpc_z = MpcControl_z_5_1(sys_z, Ts, H);
mpc_roll = MpcControl_roll_5_1(sys_roll, Ts, H);

mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

%% Simulation with changing rocket mass
x0 = zeros(12,1);
ref = @(t_, x_) ref_EPFL(t_);
Tf = 30;
rocket.mass = 1.794; % Manipulate mass for simulation
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);

% Visualize
rocket.anim_rate = 2; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title