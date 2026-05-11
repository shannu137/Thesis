function results = reconstruct_3D_from_vo(vo_data, params)

    K = [params.cam_fx,  0,             params.cam_cx;
          0,             params.cam_fy, params.cam_cy;
          0,             0,                         1];

    dist = [params.cam_k1, params.cam_k2, params.cam_p1, params.cam_p2, params.cam_k3];

    baseline = params.cam_baseline;   % metres

    N = numel(vo_data.frames);
    results(N) = struct('t',[],'track_id',[],'p_est',[],'p_true',[],'error',[]);

    for k = 1:N
        fr = vo_data.frames(k);

        if fr.n_tracks == 0
            results(k).t        = fr.t;
            results(k).track_id = [];
            results(k).p_est    = zeros(3,0);
            results(k).p_true   = zeros(3,0);
            results(k).error    = [];
            continue
        end

        % ── 1. Undistort pixel observations ─────────────────────────────
        pts_L_u = undistort_points([fr.uL(:), fr.vL(:)], K, dist);
        pts_R_u = undistort_points([fr.uR(:), fr.vR(:)], K, dist);

        uL_u = pts_L_u(:,1)';   % 1 x M
        vL_u = pts_L_u(:,2)';
        uR_u = pts_R_u(:,1)';

        % ── 2. Triangulate (rectified stereo formula) ────────────────────
        disparity = uL_u - uR_u;   % 1 x M

        % Warn about bad disparities
        bad = disparity <= 0;
        if any(bad)
            warning('Frame %d: %d/%d points have disparity <= 0 (set to NaN).', ...
                     k, sum(bad), fr.n_tracks);
        end
        disparity(bad) = NaN;

        Z =  (params.cam_fx * baseline) ./ disparity;
        X =  (uL_u - params.cam_cx) .* Z / params.cam_fx;
        Y =  (vL_u - params.cam_cy) .* Z / params.cam_fy;

        p_est = [X; Y; Z];   % 3 x M  — in LEFT camera frame

        % ── 3. Compare with ground truth ─────────────────────────────────
        % fr.p_W is in world frame; transform to left-camera frame for fair compare
        phi   = 0; theta = 0; psi = 0;   % not available per-frame here,
        % so we compare in camera frame using the simulator's depth as reference
        % (depth_L is Z in left-cam frame directly from simulator)
        depth_true = fr.depth_L;          % 1 x M  true Z

        % Simple depth error (most meaningful scalar)
        depth_error = abs(Z - depth_true);

        results(k).t        = fr.t;
        results(k).track_id = fr.track_id;
        results(k).p_est    = p_est;
        results(k).p_true   = fr.p_W;          % world frame 3xM
        results(k).depth_true = depth_true;
        results(k).depth_est  = Z;
        results(k).depth_error = depth_error;
    end
end


% ── Helpers ─────────────────────────────────────────────────────────────

function pts_u = undistort_points(pts_d, K, dist)
% Iterative undistortion of Nx2 pixel points.
    k1=dist(1); k2=dist(2); p1=dist(3); p2=dist(4); k3=dist(5);
    fx=K(1,1); fy=K(2,2); cx=K(1,3); cy=K(2,3);

    % To normalised distorted coords
    x = (pts_d(:,1) - cx) / fx;
    y = (pts_d(:,2) - cy) / fy;

    x0 = x;  y0 = y;

    for iter = 1:10
        r2       = x.^2 + y.^2;
        k_rad    = 1 + k1*r2 + k2*r2.^2 + k3*r2.^3;
        x_tan    = 2*p1.*x.*y + p2.*(r2 + 2*x.^2);
        y_tan    = p1.*(r2 + 2*y.^2) + 2*p2.*x.*y;
        x        = (x0 - x_tan) ./ k_rad;
        y        = (y0 - y_tan) ./ k_rad;
    end

    pts_u = [x*fx + cx,  y*fy + cy];
end