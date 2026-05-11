clear all; close all; clc

params = get_params();
% terrain = generate_terrain(params, true);
terrain = load("sample2.mat").terrain;

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
[Omega_t, curr_t, enc_m, curr_m, Vin] = get_enc_curr_meas(traj_interp, bekker, params, dhParams);

%%
r = params.wheel_radius;
thetadot_mod = Omega_t.*(1-s);

x0 = [traj_interp.x(:,1); traj_interp.euler(:,1)];
actual_traj = forward_kinematics(x0, traj_interp, thetadot_mod, params, dhParams, ones(6,1));

%%
[accelMeas, gyroMeas] = simulate_imu(actual_traj, params);

%%
vo_data  = simulate_stereo_vo(actual_traj, terrain, params);

%%
N = length(actual_traj.t);
figure; hold on;
for k = 1:N
    p = vo_data.frames(k).p_W;
    scatter3(p(1,:), p(2,:), p(3,:), 5, 'b', 'filled');
end
plot3(actual_traj.x(1,:), actual_traj.x(2,:), actual_traj.x(3,:), 'r-', 'LineWidth', 2);
axis equal; xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([0,4]); ylim([0,3])

figure(2);
for k = 1:N
    % ---------------- Left camera ----------------
    figure(2)
    subplot(1,2,1);
    plot(vo_data.frames(k).uL, vo_data.frames(k).vL, 'b.');
    xlim([0 params.cam_width]);
    ylim([0 params.cam_height]);   % image coordinates
    axis ij

    % ---------------- Right camera ----------------
    subplot(1,2,2);
    plot(vo_data.frames(k).uR, vo_data.frames(k).vR, 'r.');
    xlim([0 params.cam_width]);
    ylim([0 params.cam_height]);   % image coordinates
    axis ij

    pause(0.01)
end

%%
x0 = [actual_traj.x(:,1); actual_traj.euler(:,1)];
vo_result = run_stereo_vo_trajectory(vo_data, x0, params);

%%
measurements = [[vo_result.p{:}]; [vo_result.euler{:}]; Omega_t; curr_m/1000];
inputs = [gyroMeas; accelMeas; Vin];
x0 = [actual_traj.x(:,1); actual_traj.v(:,1); actual_traj.euler(:,1)];

%%
ekf = run_rover_ekf(t, inputs, measurements, vo_result.Sigma, x0, dhParams, params);

%%
s_est = zeros(6,N);
for k = 1:N
    Om = sign(ekf.x(16:21,k)) .* max(abs(ekf.x(16:21,k)),1e-3);
    vt = compute_tangential_velocity(ekf.x(:,k), gyroMeas(:,k), dhParams(k), params);
    s_est(:,k) = min(1,max(-1,1 - vt ./ (params.wheel_radius*Om)));
end
