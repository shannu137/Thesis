% ── Stereo rig ────────────────────────────────────────────────────────────
params.cam_baseline  = 0.6;                %  cm between cameras (m)
params.cam_t_offset  = [0.25; 0.3; 0.05];  % left cam in body frame
params.cam_R_offset  = singleAxisDCM(2,-pi/2)*singleAxisDCM(3,pi/2)*singleAxisDCM(1,12*pi/180);  % 10 deg down-tilt

% ── Intrinsics ────────────────────────────────────────────────────────────
params.cam_fx = 600;  params.cam_fy = 600;
params.cam_cx = 320;  params.cam_cy = 240;
params.cam_width = 640;  params.cam_height = 480;
params.cam_z_near = 0.05;

% ── FOV (should match fx/fy + image size) ────────────────────────────────
params.cam_fov_h = 2 * atan(params.cam_width  / (2*params.cam_fx));
params.cam_fov_v = 2 * atan(params.cam_height / (2*params.cam_fy));

% ── Distortion ────────────────────────────────────────────────────────────
params.cam_k1 = 0; params.cam_k2 = 0; params.cam_k3 = 0;
params.cam_p1 = 0; params.cam_p2 = 0;

% ── VO simulation ─────────────────────────────────────────────────────────
params.vo_N_feat    = 150;    % features to maintain per frame
params.vo_seed_range = 4.0;   % max ground range to seed features (m)
params.vo_sigma_px  = 0.5;    % pixel noise std dev

% ── Run ───────────────────────────────────────────────────────────────────
% terrain  = generate_terrain(params, false);
vo_data  = simulate_stereo_vo(actual_traj, terrain, params);

% ── Inspect frame k ───────────────────────────────────────────────────────
k  = 50;
fr = vo_data.frames(k);
fprintf('t=%.2fs  tracks=%d  new=%d\n', fr.t, fr.n_tracks, sum(fr.is_new));

% Stereo disparity (ideal for triangulation sanity check)
disparity = fr.uL - fr.uR;
depth_est = params.cam_fx * params.cam_baseline ./ disparity;

%%
figure; hold on;
for k = 1:N
    p = vo_data.frames(k).p_W;
    scatter3(p(1,:), p(2,:), p(3,:), 5, 'b', 'filled');
end
plot3(actual_traj.x(1,:), actual_traj.x(2,:), actual_traj.x(3,:), 'r-', 'LineWidth', 2);
axis equal; xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([0,4]); ylim([0,3])

function C = singleAxisDCM(axis, angle)
%SINGLEAXISDCM  Gives DCM corresponding to rotation about given coordinate axis by given angle

    if axis == 1
        C = [1 0 0;
             0 cos(angle) sin(angle);
             0 -sin(angle) cos(angle)];
    elseif axis == 2
            C = [cos(angle) 0 -sin(angle);
                 0 1 0;
                 sin(angle) 0 cos(angle)];
    elseif axis == 3
            C = [cos(angle) sin(angle) 0;
                 -sin(angle) cos(angle) 0;
                 0 0 1];
    else 
        error('Invalid axis specified. Specify 1 for X-axis, 2 for Y-axis, and 3 for Z-axis')
    end
end