function un_dot = forward_kinematics(us_dot, ps_dot, params, dhParams, lambda)

    a_D = params.rover_a_D;
    d_D = params.rover_d_D;
    d_S = params.rover_d_S;
    a_rho = params.rover_a_rho;
    gamma = params.rover_gamma;

    En = [eye(3,4); zeros(2,4); [0 0 0 1]];
    Es = [zeros(3,2); eye(2); [0 0]];
    S = diag([1, 1, 1, d_S^2, d_S^2, d_S^2]);

    sum_lambda_G     = zeros(4,6);
    sum_lambda_G_Jsq = zeros(4,3);
    lambda_G_Jsa_i   = zeros(4,2*6);

    for i = 1:6
        b = (-1)^i;

        J_zeta  = zeta_jacobian(params, dhParams, i);
        J_delta = delta_jacobian(params, dhParams, i);
        
        Pn = [J_zeta J_delta];
        G  = En' * (eye(6) - S*Pn*((Pn'*S*Pn)\Pn')) * S;

        Js_q = zeros(6,3);
        Js_q(:,1) = [-b*d_D; 0; b*a_D; 0; b; 0];
        if i == 1 || i == 2
            Js_q(:,2:3) = zeros(6,2);
        elseif mod(i,2) ~= 0
            Js_q(:,2) = [d_D-a_rho*sin(gamma+b*dhParams.beta); 0; a_rho*cos(gamma+b*dhParams.beta)-a_D...
                           ;0; -1; 0];
            Js_q(:,3) = zeros(6,1);
        else 
            Js_q(:,3) = [d_D-a_rho*sin(gamma+b*dhParams.beta); 0; a_rho*cos(gamma+b*dhParams.beta)-a_D...
                           ;0; -1; 0];
            Js_q(:,2) = zeros(6,1);
        end

        J_psi   = psi_jacobian(params, dhParams, i);
        J_theta = theta_jacobian(params, dhParams, i);
        Js_a    = [J_psi J_theta];

        sum_lambda_G     = sum_lambda_G + lambda(i)*G;
        sum_lambda_G_Jsq = sum_lambda_G_Jsq + lambda(i)*G*Js_q;
        lambda_G_Jsa_i(:,2*(i-1)+1:2*i) = lambda(i)*G*Js_a;
    end

    un_dot = (sum_lambda_G*En) \ ([-sum_lambda_G*Es, sum_lambda_G_Jsq, lambda_G_Jsa_i] * [us_dot; ps_dot]);
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
    beta  = dhParams.beta;
    gamma = params.rover_gamma;
    psi   = dhParams.psi(wheel_number);
    delta = dhParams.delta(wheel_number);

    d_S   = params.rover_d_S;
    a_S   = params.rover_a_S_vals(wheel_number);
    d_D   = params.rover_d_D;
    a_D   = params.rover_a_D;

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