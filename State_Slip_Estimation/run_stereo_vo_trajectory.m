function traj = run_stereo_vo_trajectory(vo_data, x0_IC, params)

    Nframes = length(vo_data.frames);

    baseline = params.cam_baseline;
    fx = params.cam_fx;
    fy = params.cam_fy;
    cx = params.cam_cx;
    cy = params.cam_cy;

    R_BC = params.cam_R_offset;
    t_BC = params.cam_t_offset;

    cameraParams = struct();
    cameraParams.R_LR = eye(3);
    cameraParams.t_LR = [-params.cam_baseline; 0; 0];
    cameraParams.K_L = [params.cam_fx  0              params.cam_cx;
                        0              params.cam_fy  params.cam_cy;
                        0              0              1];
    cameraParams.K_R = cameraParams.K_L;
    cameraParams.dist_L = [params.cam_k1, params.cam_k2, params.cam_p1, params.cam_p2];
    cameraParams.dist_R = cameraParams.dist_L;
    
    R_global = singleAxisDCM(1,x0_IC(4)) * singleAxisDCM(2,x0_IC(5)) * singleAxisDCM(3,x0_IC(6));
    p_global = x0_IC(1:3);
    Sigma_global = zeros(6,6);
    
    traj.euler = cell(Nframes,1);
    traj.p = cell(Nframes,1);
    traj.Sigma = cell(Nframes,1);
    
    % Initial pose
    traj.euler{1} = x0_IC(4:6);
    traj.p{1} = p_global;
    traj.Sigma{1} = Sigma_global;

    options.max_iter = 50;
    options.tol_x = 1e-4;
    options.tol_cost = 1e-5;
    options.verbose = false;
    
    for k = 2:Nframes

        fr_prev = vo_data.frames(k-1);
        fr_curr = vo_data.frames(k);

        [common_ids, idx_prev, idx_curr] = intersect(fr_prev.track_id, fr_curr.track_id);

        N = length(common_ids);
        if N < 6
            warning('Too few matches');
            continue;
        end
    
        uLp = fr_prev.uL(idx_prev);
        vLp = fr_prev.vL(idx_prev);
        uRp = fr_prev.uR(idx_prev);
        vRp = fr_prev.vR(idx_prev);

        uLc = fr_curr.uL(idx_curr);
        vLc = fr_curr.vL(idx_curr);
        uRc = fr_curr.uR(idx_curr);
        vRc = fr_curr.vR(idx_curr);

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

        x0.R = eye(3);
        x0.t = zeros(3,1);
        x0.P = P0;
        
        sigma_px = params.vo_sigma_px;
        W = eye(8*N) / sigma_px^2;
    
        [x_opt, info] = pose_estimation_NLWLS(y_obs, x0, W, cameraParams, options);
    
        DeltaR_C = x_opt.R;
        Deltat_C = x_opt.t;
        Sigma_C = info.Sigma_xi;

        DeltaR_B = R_BC * DeltaR_C * R_BC';
        Deltat_B = R_BC * Deltat_C + t_BC - DeltaR_B * t_BC;

        Adj_BC = [R_BC, zeros(3);
                  tilde(t_BC)*R_BC, R_BC];
        Sigma_B = Adj_BC * Sigma_C * Adj_BC';
    
        R_prev = R_global;
        p_prev = p_global;
    
        R_global = DeltaR_B * R_prev;
        p_global = p_prev - R_global'*Deltat_B;
    
        F = [eye(3), R_global' * tilde(Deltat_B) * DeltaR_B;
             zeros(3), DeltaR_B];
        
        G = [-R_global', R_global' * tilde(Deltat_B);
              zeros(3), eye(3)];
    
        Sigma_global = ...
            F * Sigma_global * F' + ...
            G * Sigma_B * G';
    
        euler_ang = rotmat2eul(R_global, 'zyx');
        euler_ang = [euler_ang(3); euler_ang(2); euler_ang(1)];

        traj.euler{k} = euler_ang;
        traj.p{k} = p_global;
        traj.Sigma{k} = Sigma_global;
    end
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