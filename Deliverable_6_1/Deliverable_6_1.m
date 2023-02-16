%% TODO: This file should produce all the plots for the deliverable
addpath(fullfile('src'));
addpath(fullfile('Deliverable_6_1'));
close all;
clear all;
clc

%% Initialize rocket and references
Ts = 1/20;
rocket = Rocket(Ts);

% MPC reference with default maximum roll = 15 deg
ref = @(t_, x_) ref_EPFL(t_);

% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref_50 = @(t_, x_) ref_EPFL(t_, roll_max);

%% Initialize controller
H = 3; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

%% Evaluate once and plot optimal open−loop trajectory
x0 = zeros(12,1);
ref4 = [2 2 2 deg2rad(50)]';

[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref4);
U_opt(:,end+1) = nan; % pad last input to get consistent size with time and state
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4);
%% Closed loop sim for ref
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
% Visualize
rocket.anim_rate = 2.5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
%% Closed loop sim for ref_50
Tf = 30;
[T, X, U, Ref_50] = rocket.simulate(x0, Tf, @nmpc.get_u, ref_50);
% Visualize
rocket.anim_rate = 2.5; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref_50);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
