clear all; close all; clc

params = get_params();
% terrain = generate_terrain(params, true);
terrain = load("terrain_sample2.mat").terrain;

start = [60,90];
goal  = [90,60];

traj = build_trajectory(start,goal,terrain,params);
plot_traj(traj)

[traj, dhParams] = get_rover_parameters(traj,terrain,params);

%%
bekker(length(traj.t)) = struct( ...
    'Fsoil',    [], ...
    'Fn_check', [], ...
    'T_wheel',  [], ...
    'theta1',   [], ...
    'theta2',   [], ...
    'z0',       [], ...
    'W_wheel',  [] );

N = length(traj.t);
for i = 1:N
    disp(i)
    s = 0.002*(i-1)*ones(6,1);
    bekker(i) = compute_bekker(traj.x(i,:)', traj.euler(i,:)', s, terrain, params, dhParams(i));
end