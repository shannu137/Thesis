clear all; close all; clc

params = get_params();
% terrain = generate_terrain(params, true);
terrain = load("terrain_sample2.mat").terrain;

%%
start = [60,90];
goal  = [90,60];

traj = generate_trajectory(start,goal,terrain,params);
%%
plot_traj(traj)

%%
t = traj.t(1):1/params.fs:traj.t(end);
[x,v,a,des_states,yaw_des] = traj.query(t);
traj_interp = struct('t', t, 'x', x', 'v', v', 'a', a', 'des_states', des_states', 'yaw_des', yaw_des');

%%
[traj_interp, dhParams] = get_dhParams(traj_interp,terrain,params);

%%
s = repmat(traj_interp.t/max(traj_interp.t),[6,1]);
bekker = get_traction_force(traj_interp, s, terrain, params, dhParams);

%%
[Omega_t, curr_t, enc_m, curr_m] = get_enc_curr_meas(traj_interp, bekker, params, dhParams);

%%
r = params.wheel_radius;
thetadot_mod = Omega_t.*(1-s);

x0 = [traj_interp.x(:,1); traj_interp.euler(:,1)];
actual_traj = forward_kinematics(x0, traj_interp, thetadot_mod, params, dhParams, ones(6,1));

%%
