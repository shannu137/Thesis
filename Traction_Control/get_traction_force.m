function bekker = get_traction_force(traj, s, terrain, params, dhParams)
    bekker(length(traj.t)) = struct( ...
        'Fsoil',    [], ...
        'Fn_check', [], ...
        'T_wheel',  [], ...
        'theta1',   [], ...
        'theta2',   [], ...
        'z0',       [], ...
        'W_wheel',  [] );

    for i = 1:length(traj.t)
        rover_pos  = traj.x(:,i);
        rover_att  = traj.euler(:,i);
        s_t        = s(:,i);
        dhParams_t = dhParams(i);

        W_total  = params.rover_mass * params.gravity;
        W_wheel  = W_total / 6;          % [N] per wheel 
    
        Fsoil_all    = zeros(6,1);
        Fn_check_all = zeros(6,1);
        T_wheel_all  = zeros(6,1);
        theta1_all   = zeros(6,1);
        theta2_all   = zeros(6,1);
        z0_all       = zeros(6,1);
        W_all        = W_wheel * ones(6,1);
    
        R_N2R = singleAxisDCM(1, rover_att(1)) * singleAxisDCM(2, rover_att(2)) * singleAxisDCM(3, rover_att(3));
        R_R2N = R_N2R';
    
        for w = 1:6
            [Fsoil_all(w), Fn_check_all(w), T_wheel_all(w), theta1_all(w), theta2_all(w), z0_all(w)] = ...
                bisect_bekker(w, W_wheel, s_t(w), rover_pos, R_R2N, terrain, params, dhParams_t);
            if(Fsoil_all(w) < 0)
                Fsoil_all(w) = 0;
            end
        end
    
        bekker(i).Fsoil    = Fsoil_all;
        bekker(i).Fn_check = Fn_check_all;
        bekker(i).T_wheel  = T_wheel_all;
        bekker(i).theta1   = theta1_all;
        bekker(i).theta2   = theta2_all;
        bekker(i).z0       = z0_all;
        bekker(i).W_wheel  = W_all;
    end
end

function [Fsoil, Fn_check, T_wheel, theta1, theta2, z0] = bisect_bekker(wheel_number, W_wheel, s, rover_pos, R_R2N, terrain, params, dhParams)

    if W_wheel <= 0
        Fsoil    = 0;
        Fn_check = 0;
        T_wheel  = 0;
        theta1   = 0;
        theta2   = 0;
        z0       = 0;
        return;
    end

    r = params.wheel_radius;
    b = params.wheel_width;

    z0_lo = 0;
    z0_hi = 0.4*r;

    [res_lo, ~,~,~,~,~,~] = bekker_residual(wheel_number, z0_lo, W_wheel, s, rover_pos, R_R2N, terrain, params, dhParams);
    [res_hi, ~,~,~,~,~,~] = bekker_residual(wheel_number, z0_hi, W_wheel, s, rover_pos, R_R2N, terrain, params, dhParams);

    if res_lo * res_hi > 0
        warning('compute_bekker: Fn(z0) does not bracket W=%.3f N. Check soil params or wheel load.', W_wheel);
        Fsoil    = 0;
        Fn_check = 0;
        T_wheel  = 0;
        theta1   = 0;
        theta2   = 0;
        z0       = 0;
        return;
    end

    N_bisect = 20;  
    tol_Fn   = 1e-3;    % [N] convergence criterion

    sigma_k   = zeros(20,1);
    tau_k     = zeros(20,1);
    theta_pts = zeros(20,1);
    w_pts     = zeros(20,1);
    th1_last  = 0;
    th2_last  = 0;

    for iter = 1:N_bisect

        z0_mid = (z0_lo + z0_hi) / 2;

        [res_mid, sk, tk, tp, wp, th1_mid, th2_mid] = bekker_residual(wheel_number, z0_mid, W_wheel, s, rover_pos, R_R2N, terrain, params, dhParams);

        sigma_k   = sk;
        tau_k     = tk;
        theta_pts = tp;
        w_pts     = wp;
        th1_last  = th1_mid;
        th2_last  = th2_mid;

        if abs(res_mid) < tol_Fn
            break;
        end

        if res_lo * res_mid < 0
            z0_hi  = z0_mid;
        else
            z0_lo  = z0_mid;
            res_lo = res_mid;
        end
    end

    z0     = (z0_lo + z0_hi) / 2;
    theta1 = th1_last;
    theta2 = th2_last;

    Fsoil = r*b*sum(w_pts.*(tau_k.*cos(theta_pts) - sigma_k.*sin(theta_pts)));

    Fn_check = r*b*sum(w_pts.*(sigma_k.*cos(theta_pts)+ tau_k.*sin(theta_pts)));

    T_wheel = r^2*b*sum(w_pts.*tau_k);

end

function [res, sigma_k, tau_k, theta_pts, w_pts, th1, th2] = bekker_residual(wheel_number, z0, W_wheel, s, rover_pos, R_R2N, terrain, params, dhParams)

    r = params.wheel_radius;
    b = params.wheel_width;
    keff = params.soil_kc + b*params.soil_kphi;  % effective stiffness

    th1 = acos(max(-1, min(1,  1 - z0 / r)));
    th2 = (2*(0.18 + 0.32*s) - 1) * th1;
    % th2 = sweep_rear_contact(wheel_number, th1, rover_pos, R_R2N, terrain, params, dhParams);

    if th1 <= 1e-6 || th2 >= th1
        sigma_k   = zeros(20,1);
        tau_k     = zeros(20,1);
        theta_pts = zeros(20,1);
        w_pts     = zeros(20,1);
        res       = -W_wheel;
        return;
    end

    [xi_gl, w_gl] = gauss_legendre_20();

    alpha     = (th1 - th2) / 2;
    beta_mid  = (th1 + th2) / 2;
    theta_pts = alpha * xi_gl + beta_mid;   % [20x1]
    w_pts     = alpha * w_gl;               % [20x1] scaled weights

    ct = cos(theta_pts);
    st = sin(theta_pts);

    z_th = max(r * (ct - cos(th1)), 0);

    sigma_k = keff * (z_th/b).^params.soil_n;
    
    j = r * ((th1 - theta_pts) - (1 - s) * (sin(th1) - st));
    j = max(j,0);
    
    tau_k = (params.soil_c + sigma_k * tan(params.soil_phi)) ...
            .* (1 - exp(-j / params.soil_K));

    Fn  = r * b * sum(w_pts .* (sigma_k .* ct + tau_k  .* st));
    res = Fn - W_wheel;

end

function theta2 = sweep_rear_contact(wheel_number, theta1, rover_pos, R_R2N, terrain, params, dhParams)
    
    N_sweep = 60;
    gap_tol = 1e-3;     % [m]

    if theta1 <= 1e-6
        theta2 = 0;
        return;
    end

    theta_sweep = linspace(0, -theta1, N_sweep);

    for k = 1:N_sweep
        th = theta_sweep(k);

        p_local = axle_to_rover(wheel_number, th, params, dhParams);
        pW = rover_pos + R_R2N * p_local;

        [z_terr, ~, ~] = terrain.query(pW(1), pW(2));

        if (pW(3) - z_terr) > gap_tol
            theta2 = (k == 1) * 0 + (k > 1) * theta_sweep(k-1);
            return;
        end
    end

    theta2 = -theta1;   % in contact all the way to rear

end

function p_local = axle_to_rover(wheel_number, theta, params, dhParams)
    r = params.wheel_radius;

    b = (-1)^wheel_number;

        % --- bogie assignment ---
        if wheel_number == 1 || wheel_number == 2
            rho   = 0;
            a_rho = 0;
        elseif wheel_number == 3 || wheel_number == 5
            rho   = dhParams.rho1;
            a_rho = params.rover_a_rho;
        else
            rho   = dhParams.rho2;
            a_rho = params.rover_a_rho;
        end
        sigma = -b*dhParams.beta + rho;

        x_local = params.rover_a_D - a_rho*cos(params.rover_gamma+b*dhParams.beta) ...
                  + params.rover_a_S_vals(wheel_number)*cos(sigma) - params.rover_d_W_vals(wheel_number)*sin(sigma) ...
                  - r*sin(sigma)*cos(theta) - r*cos(dhParams.psi(wheel_number))*cos(sigma)*sin(theta);

        y_local = -b*params.rover_d_S - r*sin(dhParams.psi(wheel_number))*sin(theta);

        z_local = params.rover_d_D - a_rho*sin(params.rover_gamma+b*dhParams.beta) ...
                  - params.rover_a_S_vals(wheel_number)*sin(sigma) - params.rover_d_W_vals(wheel_number)*cos(sigma) ...
                  - r*cos(sigma)*cos(theta) + r*cos(dhParams.psi(wheel_number))*sin(sigma)*sin(theta); 

        p_local = [x_local; y_local; z_local];
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