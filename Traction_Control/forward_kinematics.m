function actual_traj = forward_kinematics(x0, traj, thetadot_mod, params, dhParams, lambda)

    a_D = params.rover_a_D;
    d_D = params.rover_d_D;
    d_S = params.rover_d_S;
    a_rho = params.rover_a_rho;
    gamma = params.rover_gamma;

    En = [eye(3,4); zeros(2,4); [0 0 0 1]];
    Es = [zeros(3,2); eye(2); [0 0]];
    S = diag([1, 1, 1, d_S^2, d_S^2, d_S^2]);

    N = size(traj.t,2);
    un_dot = zeros(4,N);
    actual_traj = struct('v', [], 'eulerdots', []);
    
    sensed_states = traj.eulerdots(1:2,:);

    for tin = 1:N
        actuation = [dhParams(tin).psidot'; thetadot_mod(:,tin)'];
    
        us_dot = sensed_states(:,tin);
        ps_dot = [dhParams(tin).betadot; dhParams(tin).rho1dot; dhParams(tin).rho2dot; actuation(:)];
    
        sum_lambda_G = zeros(4,6);
        sum_lambda_G_Jsq = zeros(4,3);
        lambda_G_Jsa_i = zeros(4,2*6);
    
        for i = 1:6
            b = (-1)^i;

            J_zeta = zeta_jacobian(params, dhParams(i), i);
            J_delta = delta_jacobian(params, dhParams(i), i);
            
            Pn = [J_zeta J_delta];
            G = En' * (eye(6) - S*Pn*((Pn'*S*Pn)\Pn')) * S;

            Js_q(:,1) = [-b*d_D; 0; b*a_D; 0; b; 0];
            if i == 1 || i == 2
                Js_q(:,2:3) = zeros(6,2);
            elseif mod(i,2) ~= 0
                Js_q(:,2) = [d_D-a_rho*sin(gamma+b*dhParams(tin).beta); 0; a_rho*cos(gamma+b*dhParams(tin).beta)-a_D...
                               ;0; -1; 0];
                Js_q(:,3) = zeros(6,1);
            else 
                Js_q(:,3) = [d_D-a_rho*sin(gamma+b*dhParams(tin).beta); 0; a_rho*cos(gamma+b*dhParams(tin).beta)-a_D...
                               ;0; -1; 0];
                Js_q(:,2) = zeros(6,1);
            end

            J_psi = psi_jacobian(params, dhParams(i), i);
            J_theta = theta_jacobian(params, dhParams(i), i);
            Js_a = [J_psi J_theta];

            sum_lambda_G = sum_lambda_G + lambda(i)*G;
            sum_lambda_G_Jsq = sum_lambda_G_Jsq + lambda(i)*G*Js_q;
            lambda_G_Jsa_i(:,2*(i-1)+1:2*i) = lambda(i)*G*Js_a;
        end

        un_dot(:,tin) = sum_lambda_G*En \ ([-sum_lambda_G*Es, sum_lambda_G_Jsq, lambda_G_Jsa_i] * [us_dot; ps_dot]);
    end

    att = zeros(3,N);
    v = zeros(3,N);
    pos = zeros(3,N);

    t = traj.t;
    v_b       = un_dot(1:3,:);
    eulerdots = [sensed_states; un_dot(4,:)];

    for i = 1:3
        att(i,:) = x0(i+3) + cumtrapz(t,eulerdots(i,:));
    end
    
    for k = 1:N
        R_N2R = singleAxisDCM(1, att(1,k)) * singleAxisDCM(2, att(2,k)) * singleAxisDCM(3, att(3,k));
        R_R2N = R_N2R';
        v(:,k) = R_R2N*v_b(:,k);
    end

    a = zeros(3,N);
    for i = 1:3
        a(i,:) = gradient(v(i,:), t);
    end

    for i = 1:3
        pos(i,:) = x0(i) + cumtrapz(t,v(i,:));
    end

    actual_traj.t = t;
    actual_traj.x = pos;
    actual_traj.v_b = v_b;
    actual_traj.v = v;
    actual_traj.a = a;
    actual_traj.euler = att;
    actual_traj.eulerdots = eulerdots;
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