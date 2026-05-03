function [r, dr, ddr, s_grid] = build_path(wps, params)
% wps: [K x 2] (x,y) way points coordinates in terrain frame (m)

    N_eval = params.Ttotal * params.fs;
    
    % Chord length parameterization
    K = length(wps);
    s_nodes = zeros(1,K);
    for i = 2:K
        dx_i = (wps(i,1) - wps(i-1,1));
        dy_i = (wps(i,2) - wps(i-1,2));
        s_nodes(i) = s_nodes(i-1) + sqrt(dx_i^2 + dy_i^2);
    end

    % Sove for spline
    Mx = solve_spline_M(s_nodes, wps(:,1));
    My = solve_spline_M(s_nodes, wps(:,2));

    % Get path
    s_grid = linspace(s_nodes(1), s_nodes(end), N_eval)';
    r      = zeros(N_eval,2);
    dr     = zeros(N_eval,2);
    ddr    = zeros(N_eval,2);

    for k = 1:N_eval
        sc = s_grid(k);

        % find segment
        i = find(s_nodes <= sc, 1, 'last');
        if i == K
            i = K-1;
        end

        h = s_nodes(i+1) - s_nodes(i);
        A = (s_nodes(i+1) - sc) / h;
        B = (sc - s_nodes(i)) / h;

        % r(s)
        r(k,1) = A*wps(i,1) + B*wps(i+1,1) + ((A^3-A)*Mx(i) + (B^3-B)*Mx(i+1)) * h^2/6;
        r(k,2) = A*wps(i,2) + B*wps(i+1,2) + ((A^3-A)*My(i) + (B^3-B)*My(i+1)) * h^2/6;

        % r'(s)
        dAdS = -1/h;
        dBdS =  1/h;
        dr(k,1) = dAdS*wps(i,1) + dBdS*wps(i+1,1) + ((3*A^2-1)*dAdS*Mx(i) + (3*B^2-1)*dBdS*Mx(i+1)) * h^2/6;
        dr(k,2) = dAdS*wps(i,2) + dBdS*wps(i+1,2) + ((3*A^2-1)*dAdS*My(i) + (3*B^2-1)*dBdS*My(i+1)) * h^2/6;

        % r''(s)
        ddr(k,1) = A*Mx(i) + B*Mx(i+1);
        ddr(k,2) = A*My(i) + B*My(i+1);
    end
end

% Solves Tridiagonal system for cubic spline
function M = solve_spline_M(s,x)
    K = length(x);
    h = diff(s);

    mat = zeros(K,K);
    rhs = zeros(K,1);

    % BCs: Natural spline
    mat(1,1) = 1;
    mat(K,K) = 1;

    for i = 2:K-1
        mat(i,i-1) = h(i-1);
        mat(i,i)   = 2*(h(i-1) + h(i));
        mat(i,i+1) = h(i);
        rhs(i)     = 6*((x(i+1)-x(i))/h(i) - (x(i)-x(i-1))/h(i-1));
    end

    M = mat \ rhs;
end