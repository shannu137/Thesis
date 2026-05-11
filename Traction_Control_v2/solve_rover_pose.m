function [sol, contact_points] = solve_rover_pose(inputs, prev_sol, terrain, params)

    % Inputs:
    % [xR, yR, yawR, psi1..psi6]
    % Unknowns:
    % [zR, roll, pitch, beta, rho1, rho2, delta1..delta6]

    options = optimoptions('fsolve','Display','none','TolFun',1e-4,'TolX',1e-4);

    sol = fsolve(@(u) constraints(u, inputs, terrain, params), prev_sol, options);

    contact_points = compute_contacts(sol, inputs, params);

end

function F = constraints(u, inputs, terrain, params)

    zR    = u(1);
    roll  = u(2);
    pitch = u(3);
    beta  = u(4);
    rho1  = u(5);
    rho2  = u(6);
    delta = u(7:12);

    xR    = inputs(1);
    yR    = inputs(2);
    yawR  = inputs(3);
    psi   = inputs(4:9);

    F = zeros(12,1);

    % Rotation rover → world
    R_N2R = singleAxisDCM(1, roll) * singleAxisDCM(2, pitch) * singleAxisDCM(3, yawR);
    R_R2N = R_N2R';

    for i = 1:6

        b = (-1)^i;

        % --- bogie assignment ---
        if i==1 || i==2
            rho   = 0;
        elseif i==3 || i==5
            rho   = rho1;
        else
            rho   = rho2;
        end

        % --- axle Z in rover frame ---
        sigma = -b*beta + rho;
        z_axle_R = [sin(sigma); 0; cos(sigma)];
        y_axle_R = [-sin(psi(i))*cos(sigma); cos(psi(i)); sin(psi(i))*sin(sigma)];

        % --- convert to world frame ---
        z_axle = R_R2N * z_axle_R;
        y_axle = R_R2N * y_axle_R;

        % --- contact point ---
        p_local   = wheel_position_local(i, beta, rho, psi(i), delta(i), params);
        p_contact = [xR; yR; zR] + R_R2N * p_local;

        xc = p_contact(1);
        yc = p_contact(2);
        zc = p_contact(3);

        % --- terrain height ---
        [z_terr, dzdx, dzdy] = terrain.query(xc, yc);

        % --- normal from terrain ---
        n_terr = [-dzdx; -dzdy; 1];
        n_terr = n_terr / norm(n_terr);

        % --- project n onto plane normal to y_axle ---
        n_proj = n_terr - dot(n_terr, y_axle) * y_axle;
        n_proj = n_proj / norm(n_proj);

        %% CONSTRAINTS

        % (1) Contact on surface
        F(i) = (zc - z_terr);

        % (2) Orientation match
        F(i+6) = atan2(dot(cross(z_axle, n_proj), y_axle), dot(z_axle, n_proj)) - delta(i);

    end
end

function p_local = wheel_position_local(i, beta, rho, psi, delta, params)
    a_D = params.rover_a_D;
    d_D = params.rover_d_D;
    a_S = params.rover_a_S_vals(i);
    d_W = params.rover_d_W_vals(i);
    a_rho = params.rover_a_rho * ~(i == 1 || i == 2);
    r = params.wheel_radius;
    d_S = params.rover_d_S;
    gamma = params.rover_gamma;

    b = (-1)^i;
    cS = cos(-b*beta + rho);
    sS = sin(-b*beta + rho);

    p_local = [a_D + a_S*cS - d_W*sS - a_rho*cos(gamma+b*beta) - r*cos(delta)*sS - r*cos(psi)*sin(delta)*cS;
               -b*d_S - r*sin(delta)*sin(psi);
               d_D - a_S*sS - d_W*cS - a_rho*sin(gamma+b*beta) - r*cos(delta)*cS + r*cos(psi)*sin(delta)*sS];
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

function xc_all = compute_contacts(u, inputs, params)

    zR    = u(1);
    roll  = u(2);
    pitch = u(3);
    beta  = u(4);
    rho1  = u(5);
    rho2  = u(6);
    delta = u(7:12);

    xR    = inputs(1);
    yR    = inputs(2);
    yawR  = inputs(3);
    psi   = inputs(4:9);

    % Rotation rover → world
    R_N2R = singleAxisDCM(1, roll) * singleAxisDCM(2, pitch) * singleAxisDCM(3, yawR);
    R_R2N = R_N2R';

    xc_all = zeros(3,6);

    for i = 1:6

        % bogie assignment
        if i==1 || i==2
            rho = 0;
        elseif i==3 || i==5
            rho = rho1;
        else
            rho = rho2;
        end

        % contact point in rover frame
        p_local = wheel_position_local(i, beta, rho, psi(i), delta(i), params);

        % transform to world
        p_contact = [xR; yR; zR] + R_R2N * p_local;

        xc_all(:,i) = p_contact;
    end
end