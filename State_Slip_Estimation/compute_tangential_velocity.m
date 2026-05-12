function [vt, Jt] = compute_tangential_velocity(x, omega_m, dhParams, params)

    a_D = params.rover_a_D;
    d_D = params.rover_d_D;
    d_S = params.rover_d_S;
    a_rho = params.rover_a_rho;
    gamma = params.rover_gamma;
    S = diag([1, 1, 1, d_S^2, d_S^2, d_S^2]);
    a_S = params.rover_a_S_vals;
    d_W = params.rover_d_W_vals;

    rho_dot = [0, 0, dhParams.rho1dot, dhParams.rho2dot, dhParams.rho1dot, dhParams.rho2dot];

    theta_dot = zeros(6,1);
    Jt = zeros(3,6);
    
    R_N2B = singleAxisDCM(1,x(7)) * singleAxisDCM(2,x(8)) * singleAxisDCM(3,x(9));
    v_b = R_N2B * x(4:6);
    x_dot = v_b(1); y_dot = v_b(2); z_dot = v_b(3);

    wc = omega_m - x(10:12);
    B = euler_rate_matrix(x(7:9));
    euldots = B * wc;
    phix_dot = euldots(1); phiy_dot = euldots(2); phiz_dot = euldots(3);

    for i = 1:6
        b = (-1)^i;
        % ----------------- wheel speed commanded ----------------------
        J_psi = psi_jacobian(params, dhParams, i);

        X = x_dot + b*d_D*dhParams.betadot - (d_D - a_rho*sin(gamma + b*dhParams.beta))*rho_dot(i) ...
            - J_psi(1)*dhParams.psidot(i);
        Y = y_dot - J_psi(2)*dhParams.psidot(i);
        Z = - b*a_D*dhParams.betadot - (a_rho*cos(gamma + b*dhParams.beta) - a_D)*rho_dot(i)...
            - J_psi(3)*dhParams.psidot(i);
        PHI_X = phix_dot - J_psi(4)*dhParams.psidot(i);
        PHI_Y = phiy_dot - b*dhParams.betadot + rho_dot(i);
        PHI_Z = phiz_dot - J_psi(6)*dhParams.psidot(i);

        J_theta = theta_jacobian(params, dhParams, i);
        J_zeta  = zeta_jacobian(params, dhParams, i);
        J_delta = delta_jacobian(params, dhParams, i);
        z_col   = [0;0;-1;0;0;0];
    
        A = [J_theta, z_col, J_zeta, J_delta];

        chi = pinv(A'*S*A) * A' * S * [X; Y; Z; PHI_X; PHI_Y; PHI_Z];
        theta_dot(i) = chi(1);
        Jt(:,i) = J_theta(1:3);
    end
    vt = params.wheel_radius * theta_dot;
end

% =========================================== JACOBIANS ===================================================
% ========================================= psi ===========================================================
function J_psi = psi_jacobian(params, dhParams, wheel_number)

    if wheel_number == 1 || wheel_number == 2
        rho = 0;
        a_rho = 0;
    elseif mod(wheel_number,2) ~= 0
        rho = dhParams.rho1;
        a_rho = params.rover_a_rho;
    else 
        rho = dhParams.rho2;
        a_rho = params.rover_a_rho;
    end

    b = (-1)^wheel_number;
    sigma = rho - b*dhParams.beta;

    J_x = b*params.rover_d_S*cos(sigma);
    J_y = params.rover_a_S_vals(wheel_number) + params.rover_a_D*cos(sigma) - params.rover_d_D*sin(sigma) ...
          - a_rho*cos(params.rover_gamma+rho);
    J_z = -b*params.rover_d_S*sin(sigma);
    J_phix = -sin(sigma);
    J_phiz = -cos(sigma);

    J_psi = [J_x; J_y; J_z; J_phix; 0; J_phiz];
end

% ========================================= theta =========================================================
function J_theta = theta_jacobian(params, dhParams, wheel_number)

    if wheel_number == 1 || wheel_number == 2
        rho = 0;
    elseif mod(wheel_number,2) ~= 0
        rho = dhParams.rho1;
    else 
        rho = dhParams.rho2;
    end
    beta = dhParams.beta;
    psi = dhParams.psi(wheel_number);
    delta = dhParams.delta(wheel_number);
    r = params.wheel_radius;

    b = (-1)^wheel_number;
    sigma = rho - b*beta;

    J_x = r * (-sin(delta)*sin(sigma) + cos(delta)*cos(psi)*cos(sigma));
    J_y = r * (cos(delta)*sin(psi));
    J_z = r * (-sin(delta)*cos(sigma) - cos(delta)*cos(psi)*sin(sigma));

    J_theta = [J_x; J_y; J_z; 0; 0; 0];
end

% ========================================= delta =========================================================
function J_delta = delta_jacobian(params, dhParams, wheel_number)

    if wheel_number == 1 || wheel_number == 2
        rho = 0;
        a_rho = 0;
    elseif mod(wheel_number,2) ~= 0
        rho = dhParams.rho1;
        a_rho = params.rover_a_rho;
    else 
        rho = dhParams.rho2;
        a_rho = params.rover_a_rho;
    end
    beta = dhParams.beta;
    gamma = params.rover_gamma;
    psi = dhParams.psi(wheel_number);
    d_S = params.rover_d_S;
    a_S = params.rover_a_S_vals(wheel_number);
    d_D = params.rover_d_D;
    a_D = params.rover_a_D;

    b = (-1)^wheel_number;
    sigma = rho - b*beta;

    J_x = d_D*cos(psi) - a_rho*cos(psi)*sin(gamma+b*beta) - a_S*cos(psi)*sin(sigma) ...
         - params.rover_d_W_vals(wheel_number)*cos(psi)*cos(sigma) + b*d_S*sin(psi)*sin(sigma);
    J_y = -sin(psi) * (params.rover_d_W_vals(wheel_number) - d_D*cos(sigma) - a_D*sin(sigma) + a_rho*sin(gamma+rho));
    J_z = -a_D*cos(psi) + a_rho*cos(psi)*cos(gamma+b*beta) - a_S*cos(psi)*cos(sigma) ...
         + params.rover_d_W_vals(wheel_number)*cos(psi)*sin(sigma) + b*d_S*sin(psi)*cos(sigma);
    J_phix = -sin(psi) * cos(sigma);
    J_phiy = cos(psi);
    J_phiz = sin(psi) * sin(sigma);

    J_delta = [J_x; J_y; J_z; -J_phix; -J_phiy; -J_phiz];
end

% ========================================== zeta =========================================================
function J_zeta = zeta_jacobian(params, dhParams, wheel_number)

    if wheel_number == 1 || wheel_number == 2
        rho = 0;
        a_rho = 0;
    elseif mod(wheel_number,2) ~= 0
        rho = dhParams.rho1;
        a_rho = params.rover_a_rho;
    else 
        rho = dhParams.rho2;
        a_rho = params.rover_a_rho;
    end
    beta = dhParams.beta;
    gamma = params.rover_gamma;
    psi = dhParams.psi(wheel_number);
    delta = dhParams.delta(wheel_number);
    d_S = params.rover_d_S;
    a_S = params.rover_a_S_vals(wheel_number);
    d_D = params.rover_d_D;
    a_D = params.rover_a_D;

    b = (-1)^wheel_number;
    sigma = rho - b*beta;

    J_x = -b*d_S*cos(delta)*cos(sigma) + b*d_S*sin(delta)*cos(psi)*sin(sigma) ...
         + params.rover_d_W_vals(wheel_number)*sin(delta)*sin(psi)*cos(sigma) + a_rho*sin(psi)*sin(delta)*sin(gamma+b*beta) ...
         + a_S*sin(delta)*sin(psi)*sin(sigma) - d_D*sin(delta)*sin(psi);
    J_y = a_rho*cos(delta)*cos(gamma+rho) - a_rho*cos(psi)*sin(delta)*sin(gamma+rho) ...
         - a_D*cos(delta)*cos(sigma) + a_D*cos(psi)*sin(delta)*sin(sigma) ...
         + d_D*cos(delta)*sin(sigma) + d_D*cos(psi)*sin(delta)*cos(sigma) ...
         - a_S*cos(delta) - params.rover_d_W_vals(wheel_number)*cos(psi)*sin(delta);
    J_z = a_D*sin(delta)*sin(psi) + b*d_S*cos(delta)*sin(sigma) + b*d_S*cos(psi)*sin(delta)*cos(sigma) ...
         + a_S*sin(delta)*sin(psi)*cos(sigma) - a_rho*sin(delta)*sin(psi)*cos(gamma+b*beta) ...
         - params.rover_d_W_vals(wheel_number)*sin(delta)*sin(psi)*sin(sigma);
    J_phix = -cos(psi)*sin(delta)*cos(sigma) - cos(delta)*sin(sigma);
    J_phiy = -sin(delta)*sin(psi);
    J_phiz = cos(psi)*sin(delta)*sin(sigma) - cos(delta)*cos(sigma);

    J_zeta = [J_x; J_y; J_z; -J_phix; -J_phiy; -J_phiz];
end

function B = euler_rate_matrix(eul)
    phi = eul(1);
    theta = eul(2);
    
    B = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
end