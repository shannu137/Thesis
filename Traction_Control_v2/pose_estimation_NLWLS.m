function [x_opt, info] = pose_estimation_NLWLS(y_obs, x0, W, cameraParams, options)

    if nargin < 5, options = struct(); end
    if ~isfield(options, 'max_iter'), options.max_iter = 100; end
    if ~isfield(options, 'tol_x'), options.tol_x = 1e-6; end
    if ~isfield(options, 'tol_cost'), options.tol_cost = 1e-8; end
    if ~isfield(options, 'verbose'), options.verbose = true; end
    
    x = x0;
    N = size(x.P, 2);

    f = @(x) f_pose_estimation(x, cameraParams);

    r = y_obs - f(x);
    cost = r' * W * r;
    
    if options.verbose
        fprintf('\n=== Gauss-Newton Weighted Least Squares ===\n');
        fprintf('Iter\tCost\t\t||dx||\n');
        fprintf('0\t%.6e\t-\n', cost);
    end
    
    converged = false;
    for iter = 1:options.max_iter
        r = y_obs - f(x);
        R = x.R;
        t = x.t;
        P = x.P;

        S = zeros(6,6);
        d  = zeros(6,1);
        
        Ecell = cell(N,1);
        Cinv_cell = cell(N,1);
        gPcell = cell(N,1);

        for i = 1:N
            [Ji_pose, Ji_P] = J_pose_estimation(R,t,P(:,i),cameraParams);

            idx = (i-1)*8 + 1;
            ri = r(idx:idx+7, 1);
            Wi = W(idx:idx+7, idx:idx+7);

            A = Ji_pose' * Wi * Ji_pose;
            B = Ji_pose' * Wi * Ji_P;
            C = Ji_P' * Wi * Ji_P;

            gxi = Ji_pose' * Wi * ri;
            gPi = Ji_P' * Wi * ri;

            Cinv = inv(C + 1e-8*eye(3));
            E = B * Cinv;

            Cinv_cell{i} = Cinv;
            Ecell{i} = E;
            gPcell{i} = gPi;

            S = S + (A - E * B');
            d = d + (gxi - E * gPi);
        end

        lambda = 1e-6;
        delta_xi = (S + lambda*eye(6)) \ d;

        deltaP = zeros(3, N);
        for i = 1:N
            Cinv = Cinv_cell{i}; E = Ecell{i}; gP = gPcell{i};
            deltaP(:,i) = (Cinv * gP) - (E' * delta_xi);
        end
        
        cost_old = cost;

        dgamma = delta_xi(1:3);
        dt     = delta_xi(4:6);

        % Update state
        x.R = expm(-tilde(dgamma)) * x.R;
        x.t = x.t + dt;
        x.P = x.P + deltaP;

        r = y_obs - f(x);
        cost = r' * W * r;
        
        dx_norm = norm(delta_xi) + norm(deltaP(:));
        dcost = abs(cost - cost_old);
        
        if options.verbose
            fprintf('%d\t%.6e\t%.6e\n', iter, cost, dx_norm);
        end
        
        if dx_norm < options.tol_x || dcost < options.tol_cost
            converged = true;
            if options.verbose
                fprintf('Converged\n');
            end
            break;
        end
    end
    
    if ~converged && options.verbose
        fprintf('Warning: Maximum iterations reached without convergence\n');
    end

    % --------- Final covariance computation -----------
    % Final residual
    r_final = y_obs - f(x);
    
    % Residual variance estimate
    sigma2 = (r_final' * W * r_final) / (length(r_final) - 6);
    
    % Final pose covariance (6x6)
    % Covariance of [dtheta; dt]
    Sigma_xi = sigma2 * inv(S);
    
    x_opt = x;
    info.converged = converged;
    info.iterations = iter;
    info.final_cost = cost;
    info.final_residual = r_final;
    info.Sigma_xi = Sigma_xi;
end

function y_pred = f_pose_estimation(x, cameraParams)
    
    R = x.R;
    t = x.t;
    P = x.P;
    N = size(P, 2);

    R_LR = cameraParams.R_LR;  
    t_LR = cameraParams.t_LR;
    K_L  = cameraParams.K_L;
    K_R  = cameraParams.K_R;
    distL = cameraParams.dist_L;
    distR = cameraParams.dist_R;

    y_pred = zeros(8 * N, 1);

    for i = 1:N
        Pi = P(:,i);

        uLp = project_point(Pi, K_L, distL);
        uRp = project_point(R_LR * Pi + t_LR, K_R, distR);

        Pi_curr = R * Pi + t;
        uLc = project_point(Pi_curr, K_L, distL);
        uRc = project_point(R_LR * Pi_curr + t_LR, K_R, distR);

        idx = (i-1)*8 + 1;
        y_pred(idx:idx+7, 1) = [uLp; uRp; uLc; uRc];
    end
end

function u = project_point(P, K, dist)

    x = P(1)/P(3);
    y = P(2)/P(3);

    k1 = dist(1); k2 = dist(2);
    p1 = dist(3); p2 = dist(4);

    r2 = x^2 + y^2;
    radial = 1 + k1 * r2 + k2 * r2^2;

    x_tang = 2*p1*x*y + p2*(r2 + 2*x^2);
    y_tang = p1*(r2 + 2*y^2) + 2*p2*x*y;

    x_d = x * radial + x_tang;
    y_d = y * radial + y_tang;

    u = [K(1,1)*x_d + K(1,3);
         K(2,2)*y_d + K(2,3)];
end

function Jpi = J_proj(K, dist, P)
    fx = K(1,1); fy = K(2,2);
    k1 = dist(1); k2 = dist(2);
    p1 = dist(3); p2 = dist(4);

    x = P(1) / P(3);
    y = P(2) / P(3);
    r2 = x^2 + y^2;

    a = fx * ((2*k1 + 4*k2*r2)*x^2 + 6*p2*x + 2*p1*y + 1 + k1*r2 + k2*r2^2);
    b = fx* (2*p1*x + 2*p2*y + 2*x*y*(k1 + 2*k2*r2));
    c = fy* (2*p1*x + 2*p2*y + 2*x*y*(k1 + 2*k2*r2));
    d = fy * ((2*k1 + 4*k2*r2)*y^2 + 6*p1*y + 2*p2*x + 1 + k1*r2 + k2*r2^2);

    dpi_dchi = [a b; c d];
    dchi_dP = (1/P(3)) * [1 0 -x; 0 1 -y];

    Jpi = dpi_dchi * dchi_dP;
end

function [Ji_pose, Ji_P] = J_pose_estimation(R, t, Pi, cameraParams)
    R_LR = cameraParams.R_LR;
    t_LR = cameraParams.t_LR;

    Jpi_L_prev = J_proj(cameraParams.K_L, cameraParams.dist_L, Pi);
    Jpi_R_prev = J_proj(cameraParams.K_R, cameraParams.dist_R, R_LR*Pi+t_LR);

    Pc = R*Pi + t;
    Jpi_L_curr = J_proj(cameraParams.K_L, cameraParams.dist_L, Pc);
    Jpi_R_curr = J_proj(cameraParams.K_R, cameraParams.dist_R, R_LR*Pc+t_LR);

    Ji_pose = [zeros(2,6);
             zeros(2,6);
             Jpi_L_curr * [tilde(R*Pi) eye(3)];
             Jpi_R_curr * R_LR * [tilde(R*Pi) eye(3)]];

    Ji_P = [Jpi_L_prev;
            Jpi_R_prev * R_LR;
            Jpi_L_curr * R;
            Jpi_R_curr * R_LR * R];
end

function X = tilde(x)
    X = [0 -x(3) x(2);
         x(3) 0 -x(1);
         -x(2) x(1) 0];
end