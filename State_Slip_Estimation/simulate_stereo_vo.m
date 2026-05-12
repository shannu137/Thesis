function vo_data = simulate_stereo_vo(actual_traj, terrain, params)
    N = size(actual_traj.t, 2);

    dist = [params.cam_k1, params.cam_k2, ...
            params.cam_p1, params.cam_p2, params.cam_k3];

    t_L_body = params.cam_t_offset;
    t_R_body = params.cam_t_offset + ...
               params.cam_R_offset * [params.cam_baseline; 0; 0];

    pool.p_W      = zeros(3, 0);  
    pool.id       = zeros(1, 0);  
    next_id       = 1;

    vo_data.t      = actual_traj.t;
    vo_data.frames = repmat(struct( ...
    't',        [], ...
    'n_tracks', [], ...
    'track_id', [], ...
    'uL',       [], ...
    'vL',       [], ...
    'uR',       [], ...
    'vR',       [], ...
    'depth_L',  [], ...
    'p_W',      [], ...
    'is_new',   []), N, 1);

    for k = 1:N
        disp(k)
        phi   = actual_traj.euler(1, k);
        theta = actual_traj.euler(2, k);
        psi   = actual_traj.euler(3, k);

        R_WB = (singleAxisDCM(1,phi)*singleAxisDCM(2,theta)*singleAxisDCM(3,psi))';
        t_WB = actual_traj.x(:, k);

        [R_WCL, t_WCL] = body_to_world_cam(R_WB, t_WB, params.cam_R_offset, t_L_body);
        [R_WCR, t_WCR] = body_to_world_cam(R_WB, t_WB, params.cam_R_offset, t_R_body);

        n_needed = params.vo_N_feat - size(pool.p_W, 2);
        if n_needed > 0
            [new_pts, new_ids, next_id] = seed_features(n_needed, next_id, R_WCL, t_WCL, actual_traj.x(:,k), terrain, params);
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

        fr.t        = actual_traj.t(k);
        fr.n_tracks = Mk;
        fr.track_id = pool.id(vis_both);
        fr.uL       = uL_n;
        fr.vL       = vL_n;
        fr.uR       = uR_n;
        fr.vR       = vR_n;
        fr.depth_L  = dL(vis_both);
        fr.p_W      = pool.p_W(:, vis_both);
        fr.is_new   = false(1, Mk);   

        vo_data.frames(k) = fr;

        pool.p_W = pool.p_W(:, vis_both);
        pool.id  = pool.id(vis_both);
    end

    seen_ids = [];
    for k = 1:N
        ids = vo_data.frames(k).track_id;
        is_new = ~ismember(ids, seen_ids);
        vo_data.frames(k).is_new = is_new;
        seen_ids = union(seen_ids, ids);
    end
end

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