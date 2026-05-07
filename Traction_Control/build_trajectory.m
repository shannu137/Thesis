function traj = build_trajectory(start, goal, terrain, params)

    % --- Path planning ---
    wps             = get_astar_wp(start, goal, terrain, params);
    [r, dr, ddr, s] = build_path(wps, params);

    % --- TOPP ---
    [t, s, sdot, ~, ~] = topp(dr, ddr, s, params);

    % --- Ensure row vectors ---
    t    = t(:)';
    s    = s(:)';
    sdot = sdot(:)';

    % --- Smooth and differentiate ---
    sdot  = medfilt1(sdot, 5);
    sddot = (sdot(2:end).^2 - sdot(1:end-1).^2) ./ (2 * diff(s));
    sddot = medfilt1(sddot, 11);
    sddot = [sddot, sddot(end)];

    % --- Full trajectory ---
    % Desired yaw
    yaw_des = unwrap(atan2(dr(:,2), dr(:,1)));

    dx   = dr(:,1);
    dy   = dr(:,2);
    ddx  = ddr(:,1);
    ddy  = ddr(:,2);

    kappa = (dx.*ddy - dy.*ddx)./(dx.^2 + dy.^2).^(3/2);
    % Desired yaw rate
    yawdot_des = sdot' .* kappa;
    
    % world frame trajectory
    x_full = r;
    v_full = dr .* sdot';
    a_full = ddr .* (sdot'.^2) + dr .* sddot';

    % desired body frame velocities
    N = length(t);
    v_body_des = zeros(N,2);

    for k = 1:N
        cyaw = cos(yaw_des(k));
        syaw = sin(yaw_des(k));

        R_W2B = [ cyaw  syaw;
                 -syaw  cyaw ];

        v_body_des(k,:) = R_W2B * v_full(k,:)';
    end

    % --- Build interpolants once ---
    F_x1 = griddedInterpolant(t, x_full(:,1), 'pchip');
    F_x2 = griddedInterpolant(t, x_full(:,2), 'pchip');
    F_v1 = griddedInterpolant(t, v_full(:,1), 'pchip');
    F_v2 = griddedInterpolant(t, v_full(:,2), 'pchip');
    F_a1 = griddedInterpolant(t, a_full(:,1), 'pchip');
    F_a2 = griddedInterpolant(t, a_full(:,2), 'pchip');

    F_b1 = griddedInterpolant(t, v_body_des(:,1), 'pchip');
    F_b2 = griddedInterpolant(t, v_body_des(:,2), 'pchip');
    F_yaw    = griddedInterpolant(t, yaw_des, 'pchip');
    F_yawdot = griddedInterpolant(t, yawdot_des, 'pchip');

    % --- Pack struct ---
    traj.t          = t';
    traj.x          = x_full;
    traj.v          = v_full;
    traj.a          = a_full;
    traj.des_states = [v_body_des, yawdot_des];
    traj.yaw_des    = yaw_des;

    traj.query = @(t_query) query_trajectory(t_query, t, ...
                F_x1, F_x2, F_v1, F_v2, F_a1, F_a2, F_b1, F_b2, F_yaw, F_yawdot);
end

%%
function [x, v, a, v_body, yaw, yawdot] = query_trajectory(t_query, t, F_x1, F_x2, F_v1, F_v2, F_a1, F_a2)
    
    t_query = min(max(t_query(:)', t(1)), t(end));  % clamp

    x = [F_x1(t_query)', F_x2(t_query)'];
    v = [F_v1(t_query)', F_v2(t_query)'];
    a = [F_a1(t_query)', F_a2(t_query)'];
    v_body = [F_b1(t_query)', F_b2(t_query)', F_b3(t_query)'];
    yaw = F_yaw(t_query)';
    yawdot = F_yawdot(t_query)';
end