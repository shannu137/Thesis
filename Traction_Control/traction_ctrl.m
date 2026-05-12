clear all; close all; clc

params  = get_params();
terrain = load("sample2.mat").terrain;

start_pt = [60, 90];
goal_pt  = [90, 60];

% traj     = generate_trajectory(start_pt, goal_pt, terrain, params);
% plot_traj(traj)

traj = load("sample2.mat").traj;

%%
t = traj.t(1) : 1/params.fs : traj.t(end);
N = length(t);

[x_ref, v_ref, a_ref, des_states_ref, yaw_des_ref] = traj.query(t);
x_ref = x_ref'; v_ref = v_ref'; a_ref = a_ref';
des_states_ref = des_states_ref'; yaw_des_ref = yaw_des_ref';

%% Donot access: Only for simulation and validation
dh_z        = zeros(1,N);
dh_roll     = zeros(1,N);
dh_pitch    = zeros(1,N);
dh_zdot     = zeros(1,N);
dh_zddot    = zeros(1,N);
dh_rolldot  = zeros(1,N);
dh_pitchdot = zeros(1,N);

dhParams(N) = struct( ...
    'beta',     0, 'betadot',  0, 'rho1',     0, ...
    'rho1dot',  0, 'rho2',     0, 'rho2dot',  0, ...
    'psi',      zeros(6,1), 'psidot',   zeros(6,1), ...
    'delta',    zeros(6,1), 'deltadot', zeros(6,1), ...
    'thetadot_cmd', zeros(6,1) ...
);

s = repmat(t/max(t),[6,1]);

Omega_t   = zeros(6,N);
current_t = zeros(6,N);
Vin       = zeros(6,N);

alpha     = params.ema_alpha; 
alpha_der = params.ema_alpha_der; 

psi_k = zeros(6,1); delta_k = zeros(6,1);
beta_k = 0; rho1_k = 0; rho2_k = 0;
z_k = 0; zdot_k = 0; roll_k = 0; pitch_k = 0;

[z_terr, dzdx, dzdy] = terrain.query(x_ref(1,1), x_ref(2,1));
prev_sol = [z_terr + params.rover_gnd_clr; ...
            atan(dzdx); atan(dzdy); ...
            atan(dzdy) / 2; atan(dzdy) / 4; atan(dzdy) / 4; ...
            repmat(atan(dzdy), 6, 1)];

Omega_m_k  = zeros(6,1);
theta_m_k  = zeros(6,1);
i_m_k      = zeros(6,1);
e_integral = zeros(6,1);

%%
% Sensors
encoder_m  = zeros(6,N);
Omega_meas = zeros(6,1);
current_m = zeros(6,N);
accelMeas = zeros(3,N);
gyroMeas  = zeros(3,N);

vo_traj.p     = zeros(3,N);
vo_traj.euler = zeros(3,N);
vo_traj.Sigma = zeros(6,6,N);

options.max_iter = 50;
options.tol_x    = 1e-4;
options.tol_cost = 1e-5;
options.verbose  = false;

% Kinematics: Actual pose
pos       = zeros(3,N);   % [x;y;z]  actual
att       = zeros(3,N);   % [roll;pitch;yaw] actual
vel       = zeros(3,N);
vel_b     = zeros(3,N);
acc       = zeros(3,N);
eulerdots = zeros(3,N);

s_est        = zeros(6,N);
vt_filt      = zeros(6,N);
thetadot_tc  = zeros(6,N);

pos(1,1) = x_ref(1,1);
pos(2,1) = x_ref(2,1);
att(3,1) = yaw_des_ref(1);

PWM_max = 255;
Ke = params.motor_Ke;
b  = params.motor_b;
Kt = params.motor_Kt;
J  = params.motor_J;
L  = params.motor_L;
R  = params.motor_R;
r  = params.wheel_radius;

Kff_m = params.motorctrl_Kff;
Kp_m  = params.motorctrl_Kp;
Ki_m  = params.motorctrl_Ki;

dist = [params.cam_k1, params.cam_k2, ...
        params.cam_p1, params.cam_p2, params.cam_k3];

baseline = params.cam_baseline;
fx = params.cam_fx;
fy = params.cam_fy;
cx = params.cam_cx;
cy = params.cam_cy;

t_BC = params.cam_t_offset;
R_BC = params.cam_R_offset;
t_R_body = t_BC + R_BC * [baseline; 0; 0];

cameraParams = struct();
cameraParams.R_LR = eye(3);
cameraParams.t_LR = [-baseline; 0; 0];
cameraParams.K_L = [fx, 0, cx;
                    0, fy, cy;
                    0,  0,  1];
cameraParams.K_R = cameraParams.K_L;
cameraParams.dist_L = dist;
cameraParams.dist_R = cameraParams.dist_L;

pool.p_W      = zeros(3, 0);  
pool.id       = zeros(1, 0);  
next_id       = 1;

vo_frames(N) = struct( ...
't',        [], ...
'n_tracks', [], ...
'track_id', [], ...
'uL',       [], ...
'vL',       [], ...
'uR',       [], ...
'vR',       [], ...
'depth_L',  [], ...
'p_W',      [], ...
'is_new',   []);

seen_ids = [];

nx = 27;
ekf_x = zeros(nx,N);
ekf_P = cell(N,1);

ekf_state        = zeros(nx,1);
ekf_state(10:12) = params.imu.Gyroscope.ConstantBias';
ekf_state(13:15) = params.imu.Accelerometer.ConstantBias';
ekf_P{1}         = params.P0;

s_target      = 0.15 * ones(6,1);
TC_Kp         = 3.0;
TC_Ki         = 0.3;
tc_integrator = zeros(6,1);
alpha_lp      = 0.15;

%%
tic
for k = 1:N
    % -----------------------------------------------------------------------------------
    disp(k)

    if k > 1
        dt = t(k) - t(k-1);
    else
        dt = t(2) - t(1);
    end

    actual_xy_yaw = [pos(1,k); pos(2,k); att(3,k)];
    [sol, ~] = solve_rover_pose([actual_xy_yaw; psi_k], prev_sol, terrain, params);
    
    if k > 1
        dh_z(k)     = alpha*sol(1) + (1-alpha)*z_k;
        dh_roll(k)  = alpha*sol(2) + (1-alpha)*roll_k;
        dh_pitch(k) = alpha*sol(3) + (1-alpha)*pitch_k;

        dhParams(k).beta = alpha*sol(4) + (1-alpha)*beta_k;
        dhParams(k).rho1 = alpha*sol(5) + (1-alpha)*rho1_k;
        dhParams(k).rho2 = alpha*sol(6) + (1-alpha)*rho2_k;
        dhParams(k).delta = alpha*sol(7:12) + (1-alpha)*delta_k;

        raw_zdot     = (dh_z(k)     - z_k)/dt;
        raw_rolldot  = (dh_roll(k)  - roll_k)/dt;
        raw_pitchdot = (dh_pitch(k) - pitch_k)/dt;
        raw_betadot = (dhParams(k).beta   - beta_k)/dt;
        raw_rho1dot = (dhParams(k).rho1   - rho1_k)/dt;
        raw_rho2dot = (dhParams(k).rho2   - rho2_k)/dt;
        raw_deltadot = (dhParams(k).delta - delta_k)/dt;

        dh_zdot(k)     = alpha_der*raw_zdot     + (1-alpha_der)*dh_zdot(k-1);
        dh_rolldot(k)  = alpha_der*raw_rolldot  + (1-alpha_der)*dh_rolldot(k-1);
        dh_pitchdot(k) = alpha_der*raw_pitchdot + (1-alpha_der)*dh_pitchdot(k-1);
    
        dhParams(k).betadot = alpha_der*raw_betadot   + (1-alpha_der)*dhParams(k-1).betadot;
        dhParams(k).rho1dot = alpha_der*raw_rho1dot   + (1-alpha_der)*dhParams(k-1).rho1dot;
        dhParams(k).rho2dot = alpha_der*raw_rho2dot   + (1-alpha_der)*dhParams(k-1).rho2dot;
        dhParams(k).deltadot = alpha_der*raw_deltadot + (1-alpha_der)*dhParams(k-1).deltadot;

        raw_zddot = (dh_zdot(k) - zdot_k)/dt;
        dh_zddot(k)  = alpha_der*raw_zddot  + (1-alpha_der)*dh_zddot(k-1);
    else
        dh_z(k)     = sol(1);
        dh_roll(k)  = sol(2);
        dh_pitch(k) = sol(3);
    
        dhParams(k).beta  = sol(4);
        dhParams(k).rho1  = sol(5);
        dhParams(k).rho2  = sol(6);
        dhParams(k).delta = sol(7:12);

        dhParams(k).betadot  = 0;
        dhParams(k).rho1dot  = 0;
        dhParams(k).rho2dot  = 0;
        dhParams(k).deltadot = 0;
    end

    pos(3,k) = dh_z(k);
    att(1,k) = dh_roll(k);
    att(2,k) = dh_pitch(k);

    if k == 1
        vo_R_global = singleAxisDCM(1, att(1,1)) * singleAxisDCM(2, att(2,1)) * singleAxisDCM(3, att(3,1));
        vo_p_global = pos(:,1);
        vo_Sigma_global = zeros(6,6);

        vo_traj.euler(:,1) = att(:,1);
        vo_traj.p(:,1)     = pos(:,1);
        vo_traj.Sigma(:,:,1) = vo_Sigma_global;

        ekf_state(1:3)   = pos(:,1);
        ekf_state(7:9)   = att(:,1);
    end

    if k > 1
        dhParams(k).psi      = dhParams(k-1).psi;
        dhParams(k).psidot   = dhParams(k-1).psidot;
        dhParams(k).thetadot_cmd = dhParams(k-1).thetadot_cmd;
    else
        dhParams(k).psi      = zeros(6,1);
        dhParams(k).psidot   = zeros(6,1);
        dhParams(k).thetadot_cmd = zeros(6,1);
    end

    [psi_new,thetadot_cmd] = inverse_kinematics(des_states_ref(:,k), [dh_rolldot(k), dh_pitchdot(k)], params, dhParams(k));
    dhParams(k).thetadot_cmd = thetadot_cmd;

    if k > 1
        dhParams(k).psi = alpha*psi_new + (1-alpha)*psi_k;
        dpsi = wrapToPi(dhParams(k).psi - psi_k);
        dhParams(k).psidot = alpha_der*(dpsi/dt) + (1-alpha_der)*dhParams(k-1).psidot;
    else 
        dhParams(k).psi = psi_new;
    end

    psi_k = dhParams(k).psi; delta_k = dhParams(k).delta;
    beta_k = dhParams(k).beta; rho1_k = dhParams(k).rho1; rho2_k = dhParams(k).rho2;
    z_k = dh_z(k); zdot_k = dh_zdot(k); roll_k = dh_roll(k); pitch_k = dh_pitch(k);
    prev_sol = sol;

    % -----------------------------------------------------------------------------------
    % Compute vt using v(:,1) and Omega_m_k, compute slip
    F_soil = zeros(6,1);

    W_wheel = params.rover_mass * params.gravity / 6;
    R_N2R = singleAxisDCM(1, att(1,k)) * singleAxisDCM(2, att(2,k)) * singleAxisDCM(3, att(3,k));
    R_R2N = R_N2R';

    for w = 1:6
        F_soil(w) = compute_traction(w, W_wheel, s_k(w), pos, R_R2N, terrain, params, dhParams(k));
        if(F_soil(w) < 0)
            F_soil(w) = 0;
        end
    end

    % -----------------------------------------------------------------------------------
    if k == 1
        Omega_cmd = dhParams(1).thetadot_cmd;
    else
        Omega_cmd = dhParams(k).thetadot_cmd; %thetadot_tc(:,k-1);
    end

    N_motor = 50;
    dt_m = dt / N_motor;

    for mk = 1:N_motor
        e_m   = Omega_cmd - Omega_meas;
        u     = Kff_m*Omega_cmd + Kp_m*e_m + Ki_m*e_integral;
        mask  = (abs(u) < PWM_max) | (u.*e_m < 0);
        e_integral(mask) = e_integral(mask) + e_m(mask)*dt_m;
        u     = max(min(u, PWM_max), -PWM_max);
        Vin_k = (u/PWM_max) * params.motor_V_supply;

        idot     = (Vin_k - R*i_m_k - Ke*Omega_m_k) / L;
        Omegadot = (Kt*i_m_k - b*Omega_m_k - r*F_soil) / J;

        Omega_m_k = Omega_m_k + Omegadot*dt_m;
        i_m_k     = i_m_k     + idot*dt_m;
        theta_m_k = theta_m_k + Omega_m_k*dt_m;
    end

    Omega_t(:,k)   = Omega_m_k;
    current_t(:,k) = i_m_k*1000;
    current_m(:,k) = i_m_k*1000 + params.noise_curr*randn(6,1);
    encoder_m(:,k) = round(theta_m_k*params.encoder_CPR/(2*pi) + params.noise_enc*randn(6,1));
    Vin(:,k)       = Vin_k;

    % -----------------------------------------------------------------------------------
    % This part is replaced using dynamics
    % Fi = R_R2W * R_C2R * [Fsoil; 0; 0];
    % vdot = a = Sum(Fi)
    % v_b = R_W2R * v
    % xdot = v
    % I*omegadot = cross(omega,I*omega) + Sum(cross(ri,Fi))
    % Get omega and get eulerdots as B*omega
    
    thetadot_mod_k = Omega_m_k .* (1-s_k);

    actuation = [dhParams(k).psidot'; thetadot_mod_k'];
    us_dot    = [dh_rolldot(k); dh_pitchdot(k)];
    ps_dot    = [dhParams(k).betadot; dhParams(k).rho1dot; ...
                 dhParams(k).rho2dot; actuation(:)];
    un_dot    = forward_kinematics(us_dot, ps_dot, params, dhParams(k), ones(6,1));

    vel_b(:,k) = un_dot(1:3);
    eulerdots(:,k) = [dh_rolldot(k); dh_pitchdot(k); un_dot(4)];

    vel(:,k) = R_R2N * vel_b(:,k);

    if k == 1
        ekf_state(4:6) = vel(:,1);
        ekf_x(:,1)     = ekf_state;  
    end

    if k < N
        pos(1,k+1) = pos(1,k) + dt*vel(1,k);
        pos(2,k+1) = pos(2,k) + dt*vel(2,k);
        att(3,k+1) = att(3,k) + dt*eulerdots(3,k);
    end

    if k > 1
        acc(:,k) = (vel(:,k) - vel(:,k-1)) / dt;
    end

    % -----------------------------------------------------------------------------------
    T = [1,                0, -sin(dh_pitch(k));
         0,  cos(dh_roll(k)), sin(dh_roll(k))*cos(dh_pitch(k));
         0, -sin(dh_roll(k)), cos(dh_roll(k))*cos(dh_pitch(k))];
    omega_body = T*eulerdots(:,k);
    omega      = R_R2N * omega_body;

    a = acc(:,k) - [0; 0; 9.81-params.gravity];
    [accelMeas(:,k), gyroMeas(:,k)] = params.imu(a', omega', R_N2R);

    % -----------------------------------------------------------------------------------
    R_WB = R_N2R';
    t_WB = pos(:,k);

    [R_WCL, t_WCL] = body_to_world_cam(R_WB, t_WB, params.cam_R_offset, t_BC);
    [R_WCR, t_WCR] = body_to_world_cam(R_WB, t_WB, params.cam_R_offset, t_R_body);

    n_needed = params.vo_N_feat - size(pool.p_W, 2);
    if n_needed > 0
        [new_pts, new_ids, next_id] = seed_features(n_needed, next_id, R_WCL, t_WCL, pos(:,k), terrain, params);
        pool.p_W = [pool.p_W, new_pts];
        pool.id  = [pool.id,  new_ids];
    end

    [uL, vL, dL, visL] = project_features(pool.p_W, R_WCL, t_WCL, dist, params);
    [uR, vR, ~,  visR] = project_features(pool.p_W, R_WCR, t_WCR, dist, params);

    vis_both = visL & visR;

    Mk = sum(vis_both);
    uL_n = uL(vis_both) + params.vo_sigma_px * randn(1, Mk);
    vL_n = vL(vis_both) + params.vo_sigma_px * randn(1, Mk);
    uR_n = uR(vis_both) + params.vo_sigma_px * randn(1, Mk);
    vR_n = vR(vis_both) + params.vo_sigma_px * randn(1, Mk);

    ids_k  = pool.id(vis_both);
    is_new = ~ismember(ids_k, seen_ids);
    seen_ids = union(seen_ids, ids_k);

    fr.t        = t(k);
    fr.n_tracks = Mk;
    fr.track_id = pool.id(vis_both);
    fr.uL       = uL_n;
    fr.vL       = vL_n;
    fr.uR       = uR_n;
    fr.vR       = vR_n;
    fr.depth_L  = dL(vis_both);
    fr.p_W      = pool.p_W(:, vis_both);
    fr.is_new   = false(1, Mk);   
    vo_frames(k) = fr;

    pool.p_W = pool.p_W(:, vis_both);
    pool.id  = pool.id(vis_both);

    % -----------------------------------------------------------------------------------
    if k >= 2
        % -----------------------------------------------------------------------------------
        fr_prev = vo_frames(k-1);
        fr_curr = vo_frames(k);

        [common_ids, idx_prev, idx_curr] = intersect(fr_prev.track_id, fr_curr.track_id);

        Nmatches = length(common_ids);
        if Nmatches < 6
            warning('Too few matches');
            vo_traj.p(:,k)       = vo_traj.p(:,k-1);
            vo_traj.euler(:,k)   = vo_traj.euler(:,k-1);
            vo_traj.Sigma(:,:,k) = vo_traj.Sigma(:,:,k-1) + eye(6)*1e4;
            continue;
        end

        uLp = fr_prev.uL(idx_prev); vLp = fr_prev.vL(idx_prev);
        uRp = fr_prev.uR(idx_prev); vRp = fr_prev.vR(idx_prev);
        uLc = fr_curr.uL(idx_curr); vLc = fr_curr.vL(idx_curr);
        uRc = fr_curr.uR(idx_curr); vRc = fr_curr.vR(idx_curr);

        y_obs = [uLp; vLp;
                 uRp; vRp;
                 uLc; vLc;
                 uRc; vRc];
        y_obs = y_obs(:);

        disparity = uLp - uRp;
        if abs(disparity) < 1e-6
            disparity = 1e-6;
        end

        Z = fx * baseline ./ disparity;
        X = (uLp - cx) .* Z / fx;
        Y = (vLp - cy) .* Z / fy;
        P0 = [X;Y;Z];

        x0_vo.R = eye(3);
        x0_vo.t = zeros(3,1);
        x0_vo.P = P0;
        
        W_vo = eye(8*Nmatches) / params.vo_sigma_px^2;

        [x_opt, info] = pose_estimation_NLWLS(y_obs, x0_vo, W_vo, cameraParams, options);

        DeltaR_C = x_opt.R;
        Deltat_C = x_opt.t;
        Sigma_C  = info.Sigma_xi;

        DeltaR_B = R_BC * DeltaR_C * R_BC';
        Deltat_B = R_BC * Deltat_C + t_BC - DeltaR_B * t_BC;

        Adj_BC = [R_BC,               zeros(3);
                  tilde(t_BC)*R_BC, R_BC];
        Sigma_B = Adj_BC * Sigma_C * Adj_BC';

        R_prev      = vo_R_global;
        p_prev      = vo_p_global;
        vo_R_global = DeltaR_B * R_prev;
        vo_p_global = p_prev - vo_R_global' * Deltat_B;

        F_vo = [eye(3), vo_R_global' * tilde(Deltat_B) * DeltaR_B;
                zeros(3),             DeltaR_B];
        G_vo = [-vo_R_global', vo_R_global' * tilde(Deltat_B);
                 zeros(3),     eye(3)];

        vo_Sigma_global = F_vo * vo_Sigma_global * F_vo' + G_vo * Sigma_B * G_vo';

        euler_vo = rotmat2eul(vo_R_global, 'zyx');
        euler_vo = [euler_vo(3); euler_vo(2); euler_vo(1)];

        vo_traj.euler(:,k) = euler_vo;
        vo_traj.p(:,k) = vo_p_global;
        vo_traj.Sigma(:,:,k) = vo_Sigma_global;

        % -----------------------------------------------------------------------------------
        u_ekf = [gyroMeas(:,k); accelMeas(:,k); Vin(:,k)];
        [x_pred, F_ekf, G_ekf] = ekf_propagation(ekf_x(:,k-1), u_ekf, dt, dhParams(k), params);

        P_pred = F_ekf * ekf_P{k-1} * F_ekf' + G_ekf * params.Q * G_ekf' * dt;

        Omega_meas = Omega_t(:,k);
        z_ekf = [vo_traj.p(:,k); vo_traj.euler(:,k); Omega_meas; current_m(:,k)/1000];

        Sigma_vo_k = squeeze(vo_traj.Sigma(:,:,k));

        R_ekf = blkdiag(Sigma_vo_k, params.noise_enc^2  * eye(6), params.noise_curr^2 * eye(6));

        [ekf_x(:,k), ekf_P{k}] = ekf_vo_update(x_pred, P_pred, z_ekf, R_ekf);

        % -----------------------------------------------------------------------------------
        Om_est = sign(ekf_x(16:21,k)).*max(abs(ekf_x(16:21,k)),1e-3);
        vt_raw = compute_tangential_velocity(ekf_x(:,k), gyroMeas(:,k), dhParams(k), params);
        if k > 2
            vt_filt(:,k) = alpha_lp*vt_raw + (1-alpha_lp)*vt_filt(:,k-1);
        else
            vt_filt(:,k) = vt_raw;
        end
        s_est(:,k) = max(-1, min(1, 1 - vt_filt(:,k)./(r*Om_est)));
    end

    % -----------------------------------------------------------------------------------
    % If traction control use here

end
toc

% -----------------------------------------------------------------------------------------------
% Local Functions
% -----------------------------------------------------------------------------------------------
function [R_WC, t_WC] = body_to_world_cam(R_WB, t_WB, R_BC, t_C_body)
    R_WC = R_WB * R_BC;
    t_WC = R_WB * t_C_body + t_WB;
end

function [u, v, depth, visible] = project_features(p_W, R_WC, t_WC, dist, params)

    R_CW = R_WC';
    p_C  = R_CW * (p_W - t_WC);          % [3 x M]

    depth = p_C(3, :);
    front = depth > params.cam_z_near;

    u = nan(1, size(p_W,2));
    v = nan(1, size(p_W,2));

    if any(front)
        x_n = p_C(1, front) ./ p_C(3, front);
        y_n = p_C(2, front) ./ p_C(3, front);

        [x_d, y_d] = apply_distortion(x_n, y_n, dist);

        uf = params.cam_fx * x_d + params.cam_cx;
        vf = params.cam_fy * y_d + params.cam_cy;

        u(front) = uf;
        v(front) = vf;
    end

    in_fov = (u >= 0) & (u <= params.cam_width  - 1) & ...
             (v >= 0) & (v <= params.cam_height - 1);

    visible = front & in_fov;
end

function [new_pts, new_ids, next_id] = seed_features(n_needed, next_id, R_WCL, t_WCL, rover_pos, terrain, params)

    Lx = params.terrain_Lx;
    Ly = params.terrain_Ly;

    new_pts = zeros(3, 0);
    new_ids = zeros(1, 0);

    max_attempts = 10 * n_needed;   % prevent infinite loop on sparse terrain
    attempts     = 0;

    while size(new_pts, 2) < n_needed && attempts < max_attempts
        attempts = attempts + 1;

        % Random ray in normalised camera coords within half-FOV cone
        hfov = params.cam_fov_h / 2;
        vfov = params.cam_fov_v / 2;

        x_n = (2*rand - 1) * tan(hfov);
        y_n = (2*rand - 1) * tan(vfov);

        % Ray direction in world frame
        ray_C = [x_n; y_n; 1];
        ray_W = R_WCL * (ray_C / norm(ray_C));

        if ray_W(3) >= 0
            % Ray points upward — won't hit terrain
            continue
        end

        % Intersect ray with terrain using a coarse step-and-refine approach
        pt = intersect_ray_terrain(t_WCL, ray_W, terrain, params);

        if isempty(pt)
            continue
        end

        % Stay within terrain bounds
        if pt(1) < 0 || pt(1) > Lx || pt(2) < 0 || pt(2) > Ly
            continue
        end

        % Stay within seed range from rover
        if norm(pt(1:2) - rover_pos(1:2)) > params.vo_range
            continue
        end

        new_pts(:, end+1) = pt; 
        new_ids(end+1)    = next_id;
        next_id           = next_id + 1;
    end
end

function pt = intersect_ray_terrain(origin, dir, terrain, params)

    pt     = [];
    step   = 0.05;           % coarse march step (m)
    t_max  = params.vo_range * 1.5;

    t_prev = 0;
    f_prev = origin(3) - terrain.query(origin(1), origin(2));

    for t_step = step:step:t_max
        p   = origin + t_step * dir;
        if p(1) < 0 || p(1) > params.terrain_Lx || ...
           p(2) < 0 || p(2) > params.terrain_Ly
            break
        end

        [z_terr, ~, ~] = terrain.query(p(1), p(2));
        f_cur = p(3) - z_terr;

        if sign(f_cur) ~= sign(f_prev)
            % Bisect between t_prev and t_step
            ta = t_prev;  tb = t_step;
            for temp = 1:12   % 12 bisection steps → ~0.05/4096 m accuracy
                tm = (ta + tb) / 2;
                pm = origin + tm * dir;
                [zm, ~, ~] = terrain.query(pm(1), pm(2));
                if sign(pm(3) - zm) == sign(f_prev)
                    ta = tm;
                else
                    tb = tm;
                end
            end
            pm     = origin + ((ta+tb)/2) * dir;
            [zm,~,~] = terrain.query(pm(1), pm(2));
            pt     = [pm(1:2); zm];
            return
        end

        t_prev = t_step;
        f_prev = f_cur;
    end
end

function [x_d, y_d] = apply_distortion(x, y, dist)
    k1=dist(1); k2=dist(2); p1=dist(3); p2=dist(4); k3=dist(5);
    r2 = x.^2 + y.^2;
    r4 = r2.^2;
    r6 = r2.^3;
    radial = 1 + k1.*r2 + k2.*r4 + k3.*r6;
    x_d = x.*radial + 2*p1.*x.*y        + p2.*(r2 + 2*x.^2);
    y_d = y.*radial + p1.*(r2 + 2*y.^2) + 2*p2.*x.*y;
end

function X = tilde(x)
    X = [0 -x(3) x(2);
         x(3) 0 -x(1);
         -x(2) x(1) 0];
end

function angles = rotmat2eul(C,seq)
%ROTMAT2EUL  Convert DCM to Euler Angles of given sequence.

    if isequal(C*C', eye(3)) && det(C) ~= 1
        error('Invalid rotation matrix')
    end
        
    if seq == 'xyx'
        theta1 = atan2(C(1,2),-C(1,3));
        theta2 = acos(C(1,1));
        theta3 = atan2(C(2,1),C(3,1));
    elseif seq == 'xyz'
        theta1 = atan2(-C(3,2),C(3,3));
        theta2 = asin(C(3,1));
        theta3 = atan2(-C(2,1),C(1,1));
    elseif seq == 'xzx'
        theta1 = atan2(C(1,3),C(1,2));
        theta2 = acos(C(1,1));
        theta3 = atan2(C(3,1),-C(2,1));
    elseif seq == 'xzy'
        theta1 = atan2(C(2,3),C(2,2));
        theta2 = -asin(C(2,1));
        theta3 = atan2(C(3,1),C(1,1));
    elseif seq == 'yxy'
        theta1 = atan2(C(2,1),C(2,3));
        theta2 = acos(C(2,2));
        theta3 = atan2(C(1,2),-C(3,2));
    elseif seq == 'yxz'
        theta1 = atan2(C(3,1),C(3,3));
        theta2 = -asin(C(3,2));
        theta3 = atan2(C(1,2),C(2,2));
    elseif seq == 'yzx'
        theta1 = atan2(-C(1,3),C(1,1));
        theta2 = asin(C(1,2));
        theta3 = atan2(-C(3,2),C(2,2));
    elseif seq == 'yzy'
        theta1 = atan2(C(2,3),-C(2,1));
        theta2 = acos(C(2,2));
        theta3 = atan2(C(3,2),C(1,2));
    elseif seq == 'zxy'
        theta1 = atan2(-C(2,1),C(2,2));
        theta2 = asin(C(2,3));
        theta3 = atan2(-C(1,3),C(3,3));
    elseif seq == 'zxz'
        theta1 = atan2(C(3,1),-C(3,2));
        theta2 = acos(C(3,3));
        theta3 = atan2(C(1,3),C(2,3));
    elseif seq == 'zyx'
        theta1 = atan2(C(1,2),C(1,1));
        theta2 = -asin(C(1,3));
        theta3 = atan2(C(2,3),C(3,3));
    elseif seq == 'zyz'
        theta1 = atan2(C(3,2),C(3,1));
        theta2 = acos(C(3,3));
        theta3 = atan2(C(2,3),-C(1,3));
    end
    angles = [theta1, theta2,theta3];
end

function [xnew, F, G] = ekf_propagation(x, inputs, dt, dhParams, params)
    p = x(1:3);
    v = x(4:6);
    ang = x(7:9);
    bg = x(10:12);
    ba = x(13:15);
    Omega = x(16:21);
    I = x(22:27);

    wm = inputs(1:3);
    am = inputs(4:6);
    Vin = inputs(7:12);
    
    R = singleAxisDCM(1,ang(1)) * singleAxisDCM(2,ang(2)) * singleAxisDCM(3,ang(3));
    B = euler_rate_matrix(ang);
    
    wc = wm - bg;
    ac = am - ba;
    
    g = [0;0;-params.gravity];
    
    % Fres = params.c_rr * ones(6,1);
    
    pdot = v;
    vdot = -R'*ac + g;
    thetadot = B * wc;
    bgdot = zeros(3,1);
    badot = zeros(3,1);

    Ft = zeros(6,1);
    Fts = zeros(6,1);
    [vt, Jt] = compute_tangential_velocity(x, wm, dhParams,  params);
    for i = 1:6
        [Ft(i), Fts(i)] = bekker_force(x(15+i), vt(i), params);
    end

    Omegadot = zeros(6,1);
    for i = 1:6
        Omegadot(i) = (1/params.motor_J) * (params.motor_Kt*I(i) - params.motor_b*Omega(i) - params.wheel_radius*Ft(i)); % + params.r_wheel * Fres(i));
    end

    Idot = zeros(6,1);
    for i = 1:6
        Idot(i) = (1/params.motor_L) * (Vin(i) - params.motor_R*I(i) - params.motor_Ke*Omega(i));
    end
    
    xnew = x;
    
    xnew(1:3) = p + dt*pdot;
    xnew(4:6) = v + dt*vdot;
    xnew(7:9) = ang + dt*thetadot;
    xnew(10:12) = bg + dt*bgdot;
    xnew(13:15) = ba + dt*badot;
    xnew(16:21) = Omega + dt*Omegadot;
    xnew(22:27) = I + dt*Idot;

    [F, G] = compute_state_jacobians(x, am, wm, vt, Jt, Fts, params, dt);
end

function [xupd, Pupd] = ekf_vo_update(xpred, Ppred, z, R)

    p = xpred(1:3);
    ang = xpred(7:9);
    Omega = xpred(16:21);
    I = xpred(22:27);

    h = [p; ang; Omega; I];
    
    H = zeros(18,27);
    
    H(1:3,1:3) = eye(3);
    H(4:6,7:9) = eye(3);
    H(7:12,16:21) = eye(6);
    H(13:18,22:27) = eye(6);
    
    r = z - h;
    S = H * Ppred * H' + R;
    K = Ppred * H' / S;
    xupd = xpred + K * r;
    
    I27 = eye(27);
    Pupd = (I27 - K*H)*Ppred*(I27 - K*H)' + K*R*K';
end

function [F, G] = compute_state_jacobians(x, am, wm, vt, Jt, Fts, params, dt)
    
    nx = 27;
    nw = 24;

    F = eye(nx);  
    G = zeros(nx, nw);
    
    ang = x(7:9);
    Omega = x(16:21);
    phi = ang(1);
    theta = ang(2);
    psi = ang(3);
    
    R = singleAxisDCM(1,phi) * singleAxisDCM(2,theta) * singleAxisDCM(3,psi);
    
    F(1:3,4:6) = dt * eye(3);
    
    ac = am - x(13:15);
    F(4:6,7:9) = dt * R' * tilde(ac);    
    F(4:6,13:15) = dt * R';
    
    wc = wm - x(10:12);
    B = euler_rate_matrix(ang);
    F(7:9,10:12) = -dt * B;
    dBomega = [(wc(2)*cos(phi)-wc(3)*sin(phi))*tan(theta), (wc(2)*sin(phi)+wc(3)*cos(phi))/(cos(theta))^2, 0;
                -(wc(2)*sin(phi)+wc(3)*cos(phi)), 0, 0;
               (wc(2)*cos(phi)-wc(3)*sin(phi))/cos(theta), (wc(2)*sin(phi)+wc(3)*cos(phi))*sin(theta)/(cos(theta))^2, 0];
    F(7:9, 7:9) = eye(3) + dBomega * dt;

    for i = 1:6
        row = 15 + i;
        if Omega(i) == 0
            continue;
        end
        Om = sign(Omega(i)) * max(abs(Omega(i)),1e-3);
        F(row, 4:6) = dt * Fts(i) * Jt(:,i)' / (params.motor_J * Om);
        F(row,row) = 1 - dt*params.motor_b/params.motor_J - dt*Fts(i)*vt(i)/(params.motor_J*Om^2);
        F(row,21+i) = dt*params.motor_Kt/params.motor_J;
    end

    for i = 1:6
        row = 21 + i;
        F(row,row) = 1 - dt*params.motor_R/params.motor_L;
        F(row,15+i) = -dt*params.motor_Ke/params.motor_L;
    end

    G(4:6, 4:6) = -R';
    G(7:9, 1:3) = -B;
    G(10:27, 7:24) = eye(18);
end

function B = euler_rate_matrix(eul)
    phi = eul(1);
    theta = eul(2);
    
    B = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
end

function [Ft, dFtds] = bekker_force(Omega, vt, params)

    r = params.wheel_radius;
    b = params.wheel_width;
    kc   = params.soil_kc;
    kphi = params.soil_kphi;
    n    = params.soil_n;
    c    = params.soil_c;
    phi  = params.soil_phi;
    K    = params.soil_K;
    
    Ww = params.rover_mass * params.gravity / 6;
    keff = kc + b*kphi;
    z0 = (Ww^2 * b^(2*(n-1)) / (keff^2* 2 * r))^(1/(2*n+1));
    
    theta1 = acos(1 - z0/r);
    theta2 = -0.4 * theta1;
    
    [xi,w] = gauss_legendre_20();
    
    alpha = (theta1-theta2)/2;
    beta  = (theta1+theta2)/2;
    
    theta = alpha*xi + beta;
    wq    = alpha*w;
    
    ct = cos(theta);
    st = sin(theta);
    
    zth = r*(ct - cos(theta1));
    sigma = keff * (zth/b).^n;

    s = 1 - vt ./ (r*Omega + 1e-12);
    
    j = r*((theta1-theta) -(1-s)*(sin(theta1)-st));
    j = max(j,0);
    
    tau = (c + sigma*tan(phi)) .* (1 - exp(-j/K));

    Ft = r*b*sum(wq .* (tau.*ct - sigma.*st));
    
    djds = r*(sin(theta1)-st);
    dtauds = (c + sigma*tan(phi)) .* exp(-j/K) .* (djds/K);
    dFtds = r*b*sum(wq .* (dtauds .* ct));
end

function [xi, w] = gauss_legendre_20()

    xi = [ ...
        -0.993128599185094859; -0.963971927277913791; ...
        -0.912234428251325905; -0.839116971822218823; ...
        -0.746331906460150793; -0.636053680726515024; ...
        -0.510867001950827097; -0.373706088715419561; ...
        -0.227785851141645078; -0.076526521133497333; ...
         0.076526521133497333;  0.227785851141645078; ...
         0.373706088715419561;  0.510867001950827097; ...
         0.636053680726515024;  0.746331906460150793; ...
         0.839116971822218823;  0.912234428251325905; ...
         0.963971927277913791;  0.993128599185094859];

    w = [ ...
        0.017614007139152118; 0.040601429800386941; ...
        0.062672048334109064; 0.083276741576704748; ...
        0.101930119817240435; 0.118194531961518417; ...
        0.131688637831675917; 0.142096109318382051; ...
        0.149172986472603747; 0.152753387130725850; ...
        0.152753387130725850; 0.149172986472603747; ...
        0.142096109318382051; 0.131688637831675917; ...
        0.118194531961518417; 0.101930119817240435; ...
        0.083276741576704748; 0.062672048334109064; ...
        0.040601429800386941; 0.017614007139152118];

end