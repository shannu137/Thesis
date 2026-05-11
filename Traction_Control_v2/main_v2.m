clear all; close all; clc

params  = get_params();
terrain = load("sample2.mat").terrain;

%% ── 1. TRAJECTORY PLANNING ──────────────────────────────────────────────
start_pt = [60, 90];
goal_pt  = [90, 60];
% traj     = generate_trajectory(start_pt, goal_pt, terrain, params);
% plot_traj(traj)
traj = load("sample2.mat").traj;

%% ── 2. SAMPLE DESIRED STATES FROM PLANNER (xy reference + yaw_des only) ─
t = traj.t(1) : 1/params.fs : traj.t(end);
N = length(t);

[x_ref, v_ref, a_ref, des_states_ref, yaw_des_ref] = traj.query(t);
x_ref = x_ref'; v_ref = v_ref'; a_ref = a_ref';
des_states_ref = des_states_ref'; yaw_des_ref = yaw_des_ref';

%% ── 3. PREALLOCATE ───────────────────────────────────────────────────────

% dhParams-related signals (previously inside get_dhParams)
dh_z        = zeros(1, N);
dh_roll     = zeros(1, N);
dh_pitch    = zeros(1, N);
dh_zdot     = zeros(1, N);
dh_zddot    = zeros(1, N);
dh_rolldot  = zeros(1, N);
dh_pitchdot = zeros(1, N);

dhParams(N) = struct( ...
    'beta',0,'betadot',0,'rho1',0,'rho1dot',0,'rho2',0,'rho2dot',0, ...
    'psi',zeros(6,1),'psidot',zeros(6,1), ...
    'delta',zeros(6,1),'deltadot',zeros(6,1), ...
    'thetadot_cmd',zeros(6,1));

% Motor / traction
Omega_t   = zeros(6, N);
current_m = zeros(6, N);
encoder_m = zeros(6, N);
Vin       = zeros(6, N);

% Kinematics
pos       = zeros(3, N);   % [x;y;z]  actual
att       = zeros(3, N);   % [roll;pitch;yaw] actual
vel       = zeros(3, N);
vel_b     = zeros(3, N);
acc       = zeros(3, N);
eulerdots = zeros(3, N);
un_dot    = zeros(4, N);

% Sensors
accelMeas = zeros(3, N);
gyroMeas  = zeros(3, N);

% EKF
nx      = 27;
ekf_x   = zeros(nx, N);
ekf_P   = cell(N, 1);

% Slip & TC
s_est        = zeros(6, N);
vt_filt      = zeros(6, N);
thetadot_tc  = zeros(6, N);

%% ── 4. INITIAL CONDITIONS ────────────────────────────────────────────────

% Seed solve_rover_pose with terrain at start
[z_terr0, dzdx0, dzdy0] = terrain.query(x_ref(1,1), x_ref(2,1));
prev_sol = [z_terr0 + params.rover_gnd_clr;
            atan(dzdx0); atan(dzdy0);
            atan(dzdy0)/2; atan(dzdy0)/4; atan(dzdy0)/4;
            repmat(atan(dzdy0), 6, 1)];

% Initial actual position = planner start (xy only; z from terrain)
pos(1,1) = x_ref(1,1);
pos(2,1) = x_ref(2,1);
pos(3,1) = z_terr0 + params.rover_gnd_clr;
att(3,1) = yaw_des_ref(1);   % yaw from planner at k=1

% EMA filter memory (mirrors get_dhParams persistent vars)
alpha     = params.ema_alpha;
alpha_der = params.ema_alpha_der;
psi_k   = zeros(6,1); delta_k = zeros(6,1);
beta_k  = 0; rho1_k = 0; rho2_k = 0;
z_k     = 0; zdot_k = 0; roll_k = 0; pitch_k = 0;

% EKF initial state
ekf_state        = zeros(nx,1);
ekf_state(1:3)   = pos(:,1);
ekf_state(4:5)   = v_ref(:,1);
ekf_state(7:9)   = att(:,1);
ekf_state(10:12) = params.imu.Gyroscope.ConstantBias';
ekf_state(13:15) = params.imu.Accelerometer.ConstantBias';
ekf_x(:,1)       = ekf_state;
ekf_P{1}         = params.P0;

% Motor state (persistent across steps)
Omega_m_k  = zeros(6,1);
theta_m_k  = zeros(6,1);
i_m_k      = zeros(6,1);
e_integral = zeros(6,1);

% Traction control
s_target      = 0.15 * ones(6,1);
TC_Kp         = 3.0;
TC_Ki         = 0.3;
tc_integrator = zeros(6,1);
alpha_lp      = 0.15;

s_est(:,1)       = s_target;
thetadot_tc(:,1) = zeros(6,1);   % updated after first IK call

% Geometry constants for FK
a_D   = params.rover_a_D;   d_D = params.rover_d_D;
d_S   = params.rover_d_S;   a_rho = params.rover_a_rho;
gam   = params.rover_gamma;
En    = [eye(3,4); zeros(2,4); [0 0 0 1]];
Es    = [zeros(3,2); eye(2); [0 0]];
S_mat = diag([1,1,1,d_S^2,d_S^2,d_S^2]);

% Motor constants
PWM_max = 255;
Ke=params.motor_Ke; Kt=params.motor_Kt; J=params.motor_J;
L=params.motor_L;   Rm=params.motor_R;  bm=params.motor_b;
r=params.wheel_radius;
Kff=params.motorctrl_Kff; Kp_m=params.motorctrl_Kp; Ki_m=params.motorctrl_Ki;

% IMU noise
gn = params.imu.Gyroscope.NoiseDensity    / sqrt(params.fs);
an = params.imu.Accelerometer.NoiseDensity/ sqrt(params.fs);
gb = params.imu.Gyroscope.ConstantBias';
ab = params.imu.Accelerometer.ConstantBias';

%% ── 5. MAIN SEQUENTIAL LOOP ──────────────────────────────────────────────
fprintf('Sequential simulation (%d steps)...\n', N);

for k = 1:N

    if k > 1; dt = t(k)-t(k-1); else; dt = t(2)-t(1); end

    % ═══════════════════════════════════════════════════════════════
    % (a)  solve_rover_pose at ACTUAL position
    %      pos(1:2,k) = actual x,y from FK integration
    %      att(3,k)   = actual yaw
    % ═══════════════════════════════════════════════════════════════
    actual_xy_yaw = [pos(1,k); pos(2,k); att(3,k)];

    [sol, ~] = solve_rover_pose([actual_xy_yaw; psi_k], prev_sol, terrain, params);

    % ═══════════════════════════════════════════════════════════════
    % (b)  EMA filter + finite-difference derivatives
    %      (identical logic to get_dhParams, inlined)
    % ═══════════════════════════════════════════════════════════════
    if k > 1
        dh_z(k)     = alpha*sol(1)   + (1-alpha)*z_k;
        dh_roll(k)  = alpha*sol(2)   + (1-alpha)*roll_k;
        dh_pitch(k) = alpha*sol(3)   + (1-alpha)*pitch_k;

        dhParams(k).beta  = alpha*sol(4) + (1-alpha)*beta_k;
        dhParams(k).rho1  = alpha*sol(5) + (1-alpha)*rho1_k;
        dhParams(k).rho2  = alpha*sol(6) + (1-alpha)*rho2_k;
        dhParams(k).delta = alpha*sol(7:12) + (1-alpha)*delta_k;

        raw_zdot     = (dh_z(k)    - z_k)    / dt;
        raw_rolldot  = (dh_roll(k) - roll_k)  / dt;
        raw_pitchdot = (dh_pitch(k)- pitch_k) / dt;
        raw_betadot  = (dhParams(k).beta  - beta_k)  / dt;
        raw_rho1dot  = (dhParams(k).rho1  - rho1_k)  / dt;
        raw_rho2dot  = (dhParams(k).rho2  - rho2_k)  / dt;
        raw_deltadot = (dhParams(k).delta - delta_k)  / dt;

        dh_zdot(k)     = alpha_der*raw_zdot     + (1-alpha_der)*dh_zdot(k-1);
        dh_rolldot(k)  = alpha_der*raw_rolldot  + (1-alpha_der)*dh_rolldot(k-1);
        dh_pitchdot(k) = alpha_der*raw_pitchdot + (1-alpha_der)*dh_pitchdot(k-1);

        dhParams(k).betadot  = alpha_der*raw_betadot  + (1-alpha_der)*dhParams(k-1).betadot;
        dhParams(k).rho1dot  = alpha_der*raw_rho1dot  + (1-alpha_der)*dhParams(k-1).rho1dot;
        dhParams(k).rho2dot  = alpha_der*raw_rho2dot  + (1-alpha_der)*dhParams(k-1).rho2dot;
        dhParams(k).deltadot = alpha_der*raw_deltadot + (1-alpha_der)*dhParams(k-1).deltadot;

        raw_zddot    = (dh_zdot(k) - zdot_k) / dt;
        dh_zddot(k)  = alpha_der*raw_zddot + (1-alpha_der)*dh_zddot(k-1);
    else
        dh_z(k)     = sol(1);
        dh_roll(k)  = sol(2);
        dh_pitch(k) = sol(3);

        dhParams(k).beta     = sol(4);
        dhParams(k).rho1     = sol(5);
        dhParams(k).rho2     = sol(6);
        dhParams(k).delta    = sol(7:12);
        dhParams(k).betadot  = 0;
        dhParams(k).rho1dot  = 0;
        dhParams(k).rho2dot  = 0;
        dhParams(k).deltadot = zeros(6,1);
    end

    % Update actual z and attitude from terrain solution
    pos(3,k) = dh_z(k);
    att(1,k) = dh_roll(k);
    att(2,k) = dh_pitch(k);
    % att(3,k) is actual yaw, already set by FK integration below

    % ═══════════════════════════════════════════════════════════════
    % (c)  Inverse kinematics → wheel speed command
    %      Uses desired velocity states but actual roll/pitchdot
    % ═══════════════════════════════════════════════════════════════
    if k > 1
        dhParams(k).psi      = dhParams(k-1).psi;
        dhParams(k).psidot   = dhParams(k-1).psidot;
        dhParams(k).thetadot_cmd = dhParams(k-1).thetadot_cmd;
    else
        dhParams(k).psi          = zeros(6,1);
        dhParams(k).psidot       = zeros(6,1);
        dhParams(k).thetadot_cmd = zeros(6,1);
    end

    [psi_new, thetadot_cmd] = inverse_kinematics( ...
        des_states_ref(:,k), [dh_rolldot(k), dh_pitchdot(k)], params, dhParams(k));

    dhParams(k).thetadot_cmd = thetadot_cmd;

    if k > 1
        dhParams(k).psi = alpha*psi_new + (1-alpha)*psi_k;
        dpsi = wrapToPi(dhParams(k).psi - psi_k);
        dhParams(k).psidot = alpha_der*(dpsi/dt) + (1-alpha_der)*dhParams(k-1).psidot;
    else
        dhParams(k).psi = psi_new;
    end

    % Update EMA memory
    psi_k   = dhParams(k).psi;   delta_k = dhParams(k).delta;
    beta_k  = dhParams(k).beta;  rho1_k  = dhParams(k).rho1;
    rho2_k  = dhParams(k).rho2;
    z_k     = dh_z(k);   zdot_k  = dh_zdot(k);
    roll_k  = dh_roll(k); pitch_k = dh_pitch(k);
    prev_sol = sol;

    % ═══════════════════════════════════════════════════════════════
    % (d)  Bekker traction force with current slip estimate
    % ═══════════════════════════════════════════════════════════════
    s_k   = s_est(:, max(k-1,1));
    Fsoil = zeros(6,1);
    for w = 1:6
        Ow = sign(Omega_m_k(w))*max(abs(Omega_m_k(w)),1e-3);
        vt_w = 0;
        if k > 1
            vt_all = compute_tangential_velocity(ekf_x(:,k-1), gyroMeas(:,k-1), dhParams(k), params);
            vt_w   = vt_all(w);
        end
        [Fsoil(w), ~] = bekker_force_local(Ow, vt_w, s_k(w), params);
    end

    % ═══════════════════════════════════════════════════════════════
    % (e)  Motor PI controller step
    % ═══════════════════════════════════════════════════════════════
    Omega_cmd = thetadot_tc(:,max(k-1,1));
    if k == 1; Omega_cmd = dhParams(1).thetadot_cmd; end

    N_motor = 50;
    dt_m    = dt / N_motor;
    Vin_k   = zeros(6,1);

    for mk = 1:N_motor
        e_m  = Omega_cmd - Omega_m_k;
        u    = Kff*Omega_cmd + Kp_m*e_m + Ki_m*e_integral;
        mask = (abs(u) < PWM_max) | (u.*e_m < 0);
        e_integral(mask) = e_integral(mask) + e_m(mask)*dt_m;
        u     = max(min(u, PWM_max), -PWM_max);
        Vin_k = (u/PWM_max)*params.motor_V_supply;

        idot     = (Vin_k - Rm*i_m_k - Ke*Omega_m_k) / L;
        Omegadot = (Kt*i_m_k - bm*Omega_m_k - r*Fsoil) / J;

        Omega_m_k = Omega_m_k + Omegadot*dt_m;
        i_m_k     = i_m_k     + idot*dt_m;
        theta_m_k = theta_m_k + Omega_m_k*dt_m;
    end

    Omega_t(:,k)   = Omega_m_k;
    current_m(:,k) = i_m_k*1000 + params.noise_curr*randn(6,1);
    encoder_m(:,k) = round(theta_m_k*params.encoder_CPR/(2*pi) + params.noise_enc*randn(6,1));
    Vin(:,k)       = Vin_k;

    % ═══════════════════════════════════════════════════════════════
    % (f)  Forward kinematics step
    % ═══════════════════════════════════════════════════════════════
    thetadot_mod_k = Omega_m_k .* (1 - s_k);

    actuation = [dhParams(k).psidot'; thetadot_mod_k'];
    us_dot    = [dh_rolldot(k); dh_pitchdot(k)];
    ps_dot    = [dhParams(k).betadot; dhParams(k).rho1dot; ...
                 dhParams(k).rho2dot; actuation(:)];

    sum_lG     = zeros(4,6);
    sum_lG_Jsq = zeros(4,3);
    lG_Jsa     = zeros(4,12);

    for i = 1:6
        bw     = (-1)^i;
        Jzeta  = zeta_jac(params, dhParams(k), i);
        Jdelta = delta_jac(params, dhParams(k), i);
        Pn     = [Jzeta Jdelta];
        G      = En'*(eye(6) - S_mat*Pn*((Pn'*S_mat*Pn)\Pn'))*S_mat;

        Jsq = zeros(6,3);
        Jsq(:,1) = [-bw*d_D; 0; bw*a_D; 0; bw; 0];
        if ~(i==1||i==2)
            col = 2 + (mod(i,2)==0);
            Jsq(:,col) = [d_D - a_rho*sin(gam+bw*dhParams(k).beta); 0; ...
                          a_rho*cos(gam+bw*dhParams(k).beta)-a_D; 0; -1; 0];
        end

        Jps = psi_jac(params, dhParams(k), i);
        Jth = theta_jac(params, dhParams(k), i);

        sum_lG     = sum_lG + G;
        sum_lG_Jsq = sum_lG_Jsq + G*Jsq;
        lG_Jsa(:, 2*(i-1)+1:2*i) = G*[Jps Jth];
    end

    un_dot(:,k) = sum_lG*En \ ...
        ([-sum_lG*Es, sum_lG_Jsq, lG_Jsa]*[us_dot; ps_dot]);

    vel_b(:,k)     = un_dot(1:3,k);
    eulerdots(:,k) = [dh_rolldot(k); dh_pitchdot(k); un_dot(4,k)];

    % Rotate body velocity to world
    R3 = dcm_local(att(:,k));
    vel(:,k) = R3' * vel_b(:,k);

    % Integrate position and yaw for NEXT step
    if k < N
        pos(1,k+1) = pos(1,k) + dt*vel(1,k);
        pos(2,k+1) = pos(2,k) + dt*vel(2,k);
        % pos(3,k+1) comes from terrain at next step
        att(3,k+1) = att(3,k) + dt*eulerdots(3,k);
    end

    if k > 1
        acc(:,k) = (vel(:,k) - vel(:,k-1)) / dt;
    end

    % ═══════════════════════════════════════════════════════════════
    % (g)  IMU simulation
    % ═══════════════════════════════════════════════════════════════
    gyroMeas(:,k) = eulerdots(:,k) + gb + gn*randn(3,1);
    if k > 1
        accelMeas(:,k) = R3*(acc(:,k) - [0;0;-params.gravity]) + ab + an*randn(3,1);
    else
        accelMeas(:,k) = ab + an*randn(3,1);
    end

    % ═══════════════════════════════════════════════════════════════
    % (h)  EKF propagation + update
    % ═══════════════════════════════════════════════════════════════
    if k > 1
        u_ekf  = [gyroMeas(:,k); accelMeas(:,k); Vin(:,k)];
        [x_pred, F_k, G_k] = ekf_prop(ekf_x(:,k-1), u_ekf, dt, dhParams(k), params);
        P_pred = F_k*ekf_P{k-1}*F_k' + G_k*params.Q*G_k'*dt;

        z_ekf = [pos(:,k); att(:,k); Omega_t(:,k); current_m(:,k)/1000];
        R_ekf = blkdiag(params.noise_enc^2*eye(3), ...
                        params.noise_enc^2*eye(3), ...
                        params.noise_enc^2*eye(6), ...
                        params.noise_curr^2*eye(6));

        [ekf_x(:,k), ekf_P{k}] = ekf_update(x_pred, P_pred, z_ekf, R_ekf);
    end

    % ═══════════════════════════════════════════════════════════════
    % (i)  Slip estimation from EKF
    % ═══════════════════════════════════════════════════════════════
    if k > 1
        Om_est = sign(ekf_x(16:21,k)).*max(abs(ekf_x(16:21,k)),1e-3);
        vt_raw = compute_tangential_velocity(ekf_x(:,k), gyroMeas(:,k), dhParams(k), params);
        if k > 2
            vt_filt(:,k) = alpha_lp*vt_raw + (1-alpha_lp)*vt_filt(:,k-1);
        else
            vt_filt(:,k) = vt_raw;
        end
        s_est(:,k) = max(-1, min(1, 1 - vt_filt(:,k)./(r*Om_est)));
    else
        s_est(:,k) = s_target;
    end

    % ═══════════════════════════════════════════════════════════════
    % (j)  Traction control PI → wheel speed command for next step
    % ═══════════════════════════════════════════════════════════════
    e_tc          = s_est(:,k) - s_target;
    tc_integrator = clamp_vec(tc_integrator + e_tc*dt, -1, 1);
    delta_omega   = -(TC_Kp*e_tc + TC_Ki*tc_integrator);
    thetadot_tc(:,k) = dhParams(k).thetadot_cmd + delta_omega;

    if mod(k,50)==0
        fprintf('  k=%4d/%d | pos=[%.2f, %.2f, %.2f] | slip=%.3f | |e_tc|=%.3f\n', ...
            k, N, pos(1,k), pos(2,k), pos(3,k), ...
            mean(abs(s_est(:,k))), mean(abs(e_tc)));
    end
end

fprintf('Done.\n');

%% ── 6. PACK TRAJECTORY ───────────────────────────────────────────────────
actual_traj.t         = t;
actual_traj.x         = pos;
actual_traj.v         = vel;
actual_traj.v_b       = vel_b;
actual_traj.a         = acc;
actual_traj.euler     = att;
actual_traj.eulerdots = eulerdots;

%% ── 7. PLOTS ─────────────────────────────────────────────────────────────
wlbl = {'FL','FR','ML','MR','RL','RR'};

figure;
plot3(x_ref(1,:), x_ref(2,:), x_ref(3,:), 'k--','LineWidth',1.5); hold on;
plot3(pos(1,:),   pos(2,:),   pos(3,:),   'b-','LineWidth',2);
legend('Planned path','Actual path (TC)','Location','best');
xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal; grid on;
title('Planned vs Actual Trajectory');

figure;
for w = 1:6
    subplot(3,2,w);
    plot(t, s_est(w,:),'b-','LineWidth',1.2); hold on;
    yline(s_target(w),'r--','\lambda^*');
    ylim([-0.2 1.0]); xlabel('t (s)'); ylabel('\lambda');
    title(['Wheel ' wlbl{w}]); grid on;
end
sgtitle('Per-wheel slip (actual-path sequential)');

figure;
for w = 1:6
    subplot(3,2,w);
    plot(t, arrayfun(@(k) dhParams(k).thetadot_cmd(w),1:N), 'k--','LineWidth',1); hold on;
    plot(t, thetadot_tc(w,:), 'b-', 'LineWidth',1.2);
    plot(t, Omega_t(w,:),     'r-', 'LineWidth',1);
    legend('IK cmd','TC cmd','Actual','Location','best');
    xlabel('t (s)'); ylabel('rad/s'); title(['Wheel ' wlbl{w}]); grid on;
end
sgtitle('Wheel speeds: IK command vs TC vs actual');

figure;
subplot(3,1,1); plot(t, pos(1,:),'b-'); hold on; plot(t, x_ref(1,:),'k--');
legend('Actual','Planned'); ylabel('x (m)'); grid on; title('Position tracking');
subplot(3,1,2); plot(t, pos(2,:),'b-'); hold on; plot(t, x_ref(2,:),'k--');
ylabel('y (m)'); grid on;
subplot(3,1,3); plot(t, att(3,:)*180/pi,'b-'); hold on; plot(t, yaw_des_ref*180/pi,'k--');
ylabel('yaw (deg)'); xlabel('t (s)'); grid on;

%% ═══════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
% ═══════════════════════════════════════════════════════════════════════

function out = clamp_vec(x, lo, hi)
    out = max(lo, min(hi, x));
end

function R = dcm_local(att)
    R = sax(1,att(1))*sax(2,att(2))*sax(3,att(3));
end

function C = sax(axis, angle)
    ca=cos(angle); sa=sin(angle);
    if axis==1,     C=[1 0 0;0 ca sa;0 -sa ca];
    elseif axis==2, C=[ca 0 -sa;0 1 0;sa 0 ca];
    else,           C=[ca sa 0;-sa ca 0;0 0 1]; end
end

function B = eul_rate(eul)
    phi=eul(1); th=eul(2);
    B=[1 sin(phi)*tan(th) cos(phi)*tan(th);
       0 cos(phi)        -sin(phi);
       0 sin(phi)/cos(th) cos(phi)/cos(th)];
end

function X = tilde_v(x)
    X=[0 -x(3) x(2);x(3) 0 -x(1);-x(2) x(1) 0];
end

% ── Bekker (explicit slip) ────────────────────────────────────────────────
function [Ft, dFtds] = bekker_force_local(Omega, vt, s, params)
    r=params.wheel_radius; bw=params.wheel_width;
    kc=params.soil_kc; kphi=params.soil_kphi; n=params.soil_n;
    c=params.soil_c; phi=params.soil_phi; K=params.soil_K;
    Ww=params.rover_mass*params.gravity/6;
    keff=kc+bw*kphi;
    z0=(Ww^2*bw^(2*(n-1))/(keff^2*2*r))^(1/(2*n+1));
    th1=acos(1-z0/r); th2=-0.4*th1;
    [xi,w]=gl20();
    ag=(th1-th2)/2; bg_=(th1+th2)/2;
    th=ag*xi+bg_; wq=ag*w;
    ct=cos(th); st=sin(th);
    zth=r*(ct-cos(th1));
    sigma=keff*(zth/bw).^n;
    j=r*((th1-th)-(1-s)*(sin(th1)-st)); j=max(j,0);
    tau=(c+sigma*tan(phi)).*(1-exp(-j/K));
    Ft=r*bw*sum(wq.*(tau.*ct-sigma.*st));
    djds=r*(sin(th1)-st);
    dtauds=(c+sigma*tan(phi)).*exp(-j/K).*(djds/K);
    dFtds=r*bw*sum(wq.*(dtauds.*ct));
end

% ── EKF propagation ───────────────────────────────────────────────────────
function [xnew,F,G] = ekf_prop(x, inputs, dt, dhp, params)
    v_=x(4:6); ang=x(7:9); bg=x(10:12); ba=x(13:15);
    Omega=x(16:21); Ic=x(22:27);
    wm=inputs(1:3); am=inputs(4:6); Vin_=inputs(7:12);
    R_=sax(1,ang(1))*sax(2,ang(2))*sax(3,ang(3));
    B_=eul_rate(ang); wc=wm-bg; ac=am-ba;
    g_=[0;0;-params.gravity];
    pdot=v_; vdot=-R_'*ac+g_; tdot=B_*wc;
    [vt,Jt]=compute_tangential_velocity(x,wm,dhp,params);
    Om_s=sign(Omega).*max(abs(Omega),1e-3);
    s_c=max(-1,min(1,1-vt./(params.wheel_radius*Om_s)));
    Ft=zeros(6,1); Fts=zeros(6,1);
    for i=1:6
        [Ft(i),Fts(i)]=bekker_force_local(Om_s(i),vt(i),s_c(i),params);
    end
    Od=zeros(6,1); Id=zeros(6,1);
    for i=1:6
        Od(i)=(params.motor_Kt*Ic(i)-params.motor_b*Omega(i)-params.wheel_radius*Ft(i))/params.motor_J;
        Id(i)=(Vin_(i)-params.motor_R*Ic(i)-params.motor_Ke*Omega(i))/params.motor_L;
    end
    xnew=x;
    xnew(1:3)=x(1:3)+dt*pdot; xnew(4:6)=v_+dt*vdot;
    xnew(7:9)=ang+dt*tdot;
    xnew(16:21)=Omega+dt*Od; xnew(22:27)=Ic+dt*Id;
    [F,G]=state_jac(x,am,wm,vt,Jt,Fts,params,dt);
end

function [F,G]=state_jac(x,am,wm,vt,Jt,Fts,params,dt)
    nx=27; F=eye(nx); G=zeros(nx,24);
    ang=x(7:9); Omega=x(16:21);
    R_=sax(1,ang(1))*sax(2,ang(2))*sax(3,ang(3));
    B_=eul_rate(ang);
    F(1:3,4:6)=dt*eye(3);
    ac=am-x(13:15);
    F(4:6,7:9)=dt*R_'*tilde_v(ac); F(4:6,13:15)=dt*R_';
    wc=wm-x(10:12);
    F(7:9,10:12)=-dt*B_;
    phi=ang(1); th=ang(2);
    dB=[(wc(2)*cos(phi)-wc(3)*sin(phi))*tan(th),(wc(2)*sin(phi)+wc(3)*cos(phi))/(cos(th))^2,0;
        -(wc(2)*sin(phi)+wc(3)*cos(phi)),0,0;
        (wc(2)*cos(phi)-wc(3)*sin(phi))/cos(th),(wc(2)*sin(phi)+wc(3)*cos(phi))*sin(th)/(cos(th))^2,0];
    F(7:9,7:9)=eye(3)+dB*dt;
    for i=1:6
        rw=15+i; if Omega(i)==0; continue; end
        Om=sign(Omega(i))*max(abs(Omega(i)),1e-3);
        F(rw,4:6)=dt*Fts(i)*Jt(:,i)'/(params.motor_J*Om);
        F(rw,rw)=1-dt*params.motor_b/params.motor_J-dt*Fts(i)*vt(i)/(params.motor_J*Om^2);
        F(rw,21+i)=dt*params.motor_Kt/params.motor_J;
    end
    for i=1:6
        rw=21+i;
        F(rw,rw)=1-dt*params.motor_R/params.motor_L;
        F(rw,15+i)=-dt*params.motor_Ke/params.motor_L;
    end
    G(4:6,4:6)=-R_'; G(7:9,1:3)=-B_; G(10:27,7:24)=eye(18);
end

function [xu,Pu]=ekf_update(xp,Pp,z,Rn)
    H=zeros(18,27);
    H(1:3,1:3)=eye(3); H(4:6,7:9)=eye(3);
    H(7:12,16:21)=eye(6); H(13:18,22:27)=eye(6);
    inn=z-[xp(1:3);xp(7:9);xp(16:21);xp(22:27)];
    S_=H*Pp*H'+Rn; K=Pp*H'/S_;
    xu=xp+K*inn;
    I27=eye(27); Pu=(I27-K*H)*Pp*(I27-K*H)'+K*Rn*K';
end

% ── FK Jacobians (from forward_kinematics.m) ─────────────────────────────
function J=psi_jac(params,dhp,i)
    if i<=2; rho=0;a_rho=0;
    elseif mod(i,2)~=0; rho=dhp.rho1;a_rho=params.rover_a_rho;
    else; rho=dhp.rho2;a_rho=params.rover_a_rho; end
    b=(-1)^i; sig=rho-b*dhp.beta;
    J=[b*params.rover_d_S*cos(sig);
       params.rover_a_S_vals(i)+params.rover_a_D*cos(sig)-params.rover_d_D*sin(sig)-a_rho*cos(params.rover_gamma+rho);
       -b*params.rover_d_S*sin(sig); -sin(sig); 0; -cos(sig)];
end

function J=theta_jac(params,dhp,i)
    if i<=2; rho=0;
    elseif mod(i,2)~=0; rho=dhp.rho1;
    else; rho=dhp.rho2; end
    b=(-1)^i; sig=rho-b*dhp.beta;
    psi=dhp.psi(i); del=dhp.delta(i); r=params.wheel_radius;
    J=r*[-sin(del)*sin(sig)+cos(del)*cos(psi)*cos(sig);
          cos(del)*sin(psi);
         -sin(del)*cos(sig)-cos(del)*cos(psi)*sin(sig); 0;0;0];
end

function J=delta_jac(params,dhp,i)
    if i<=2; rho=0;a_rho=0;
    elseif mod(i,2)~=0; rho=dhp.rho1;a_rho=params.rover_a_rho;
    else; rho=dhp.rho2;a_rho=params.rover_a_rho; end
    b=(-1)^i; sig=rho-b*dhp.beta; gam=params.rover_gamma;
    psi=dhp.psi(i); dS=params.rover_d_S; aS=params.rover_a_S_vals(i);
    dD=params.rover_d_D; aD=params.rover_a_D; dW=params.rover_d_W_vals(i);
    Jx=dD*cos(psi)-a_rho*cos(psi)*sin(gam+b*dhp.beta)-aS*cos(psi)*sin(sig)-dW*cos(psi)*cos(sig)+b*dS*sin(psi)*sin(sig);
    Jy=-sin(psi)*(dW-dD*cos(sig)-aD*sin(sig)+a_rho*sin(gam+rho));
    Jz=-aD*cos(psi)+a_rho*cos(psi)*cos(gam+b*dhp.beta)-aS*cos(psi)*cos(sig)+dW*cos(psi)*sin(sig)+b*dS*sin(psi)*cos(sig);
    Jp=-sin(psi)*cos(sig); Jpy=cos(psi); Jpz=sin(psi)*sin(sig);
    J=[Jx;Jy;Jz;-Jp;-Jpy;-Jpz];
end

function J=zeta_jac(params,dhp,i)
    if i<=2; rho=0;a_rho=0;
    elseif mod(i,2)~=0; rho=dhp.rho1;a_rho=params.rover_a_rho;
    else; rho=dhp.rho2;a_rho=params.rover_a_rho; end
    b=(-1)^i; sig=rho-b*dhp.beta; gam=params.rover_gamma;
    psi=dhp.psi(i); del=dhp.delta(i);
    dS=params.rover_d_S; aS=params.rover_a_S_vals(i);
    dD=params.rover_d_D; aD=params.rover_a_D; dW=params.rover_d_W_vals(i);
    Jx=-b*dS*cos(del)*cos(sig)+b*dS*sin(del)*cos(psi)*sin(sig)+dW*sin(del)*sin(psi)*cos(sig)+a_rho*sin(psi)*sin(del)*sin(gam+b*dhp.beta)+aS*sin(del)*sin(psi)*sin(sig)-dD*sin(del)*sin(psi);
    Jy=a_rho*cos(del)*cos(gam+rho)-a_rho*cos(psi)*sin(del)*sin(gam+rho)-aD*cos(del)*cos(sig)+aD*cos(psi)*sin(del)*sin(sig)+dD*cos(del)*sin(sig)+dD*cos(psi)*sin(del)*cos(sig)-aS*cos(del)-dW*cos(psi)*sin(del);
    Jz=aD*sin(del)*sin(psi)+b*dS*cos(del)*sin(sig)+b*dS*cos(psi)*sin(del)*cos(sig)+aS*sin(del)*sin(psi)*cos(sig)-a_rho*sin(del)*sin(psi)*cos(gam+b*dhp.beta)-dW*sin(del)*sin(psi)*sin(sig);
    Jp=-cos(psi)*sin(del)*cos(sig)-cos(del)*sin(sig);
    Jpy=-sin(del)*sin(psi);
    Jpz=cos(psi)*sin(del)*sin(sig)-cos(del)*cos(sig);
    J=[Jx;Jy;Jz;-Jp;-Jpy;-Jpz];
end

function [xi,w]=gl20()
    xi=[-0.993128599185094859;-0.963971927277913791;-0.912234428251325905;-0.839116971822218823;-0.746331906460150793;-0.636053680726515024;-0.510867001950827097;-0.373706088715419561;-0.227785851141645078;-0.076526521133497333;0.076526521133497333;0.227785851141645078;0.373706088715419561;0.510867001950827097;0.636053680726515024;0.746331906460150793;0.839116971822218823;0.912234428251325905;0.963971927277913791;0.993128599185094859];
    w=[0.017614007139152118;0.040601429800386941;0.062672048334109064;0.083276741576704748;0.101930119817240435;0.118194531961518417;0.131688637831675917;0.142096109318382051;0.149172986472603747;0.152753387130725850;0.152753387130725850;0.149172986472603747;0.142096109318382051;0.131688637831675917;0.118194531961518417;0.101930119817240435;0.083276741576704748;0.062672048334109064;0.040601429800386941;0.017614007139152118];
end