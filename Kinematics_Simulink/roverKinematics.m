% roverParams = [d_D, a_D, d_S, a_rho, gamma, wheel_radius];
roverParams = [ ...
    0, ...                               % d_D
    0, ...                               % a_D
    (18.562 + 13.176) * 1e-02, ...       % d_S
    sqrt(12.454^2 + 3.3492^2) * 1e-02, ... % a_rho
    atan2(3.3492, 12.454), ...           % gamma
    6e-2 ...                             % wheel radius
];

% dhParams = [beta, rho1, rho2, beta_dot, rho1_dot, rho2_dot];
dhParams = [0, 0, 0, 0, 0, 0];

% wheels: each column is a wheel
% rows: [a_S; d_W; psi; theta; zeta; eta; delta; psi_dot; theta_dot; zeta_dot; eta_dot; delta_dot]
wheels = zeros(12,6);

a_S_vals = [25.1, 25.1, 12.568, 12.568, -12.499, -12.499]*1e-2;
d_W_vals = [(10.408+8.1978), (10.408+8.1978), (7.0577+8.1978), ...
             (7.0577+8.1978), (7.0577+8.1978), (7.0577+8.1978)]*1e-2;

wheels(1,:) = a_S_vals;
wheels(2,:) = d_W_vals;
% rest already initialized to zero

d_S = roverParams(3);
S = diag([1, 1, 1, d_S^2, d_S^2, d_S^2]);

[psi, theta_dot] = inverse_kinematics([0.1, -0, 0.25], [0,0], roverParams, dhParams, wheels);
for i = 1:6
wheels(3,i) = psi(i);
end
disp(psi*180/pi)
disp(theta_dot*60/(2*pi))
forward_kinematics([0;0], [zeros(6,1)'; theta_dot'], roverParams, dhParams, wheels, ones(6,1))

%% ================= PARAMETERS ======================
dt = 0.1;          % timestep [s]
T  = 200;           % total simulation time [s]
N  = round(T/dt);  

% Preallocate logs
x     = zeros(1,N);  y     = zeros(1,N);  z     = zeros(1,N);
phi_z = zeros(1,N);
xdot  = zeros(1,N);  ydot  = zeros(1,N);  zdot  = zeros(1,N);
phizdot = zeros(1,N);

% ================= DESIRED TRAJECTORY ======================
tvec = (0:N-1)*dt;
a = 1.0; 
b = 0.5;       % ellipse radii [m]
omega = 0.1;   % angular speed [rad/s]

% Desired path in global frame
x_d = a*cos(omega*tvec);
y_d = b*sin(omega*tvec);

% Desired velocities in global frame
x_dot_d = gradient(x_d, dt);
y_dot_d = gradient(y_d, dt);

% Heading = direction of velocity (not position angle!)
phi_z_d = atan2(y_dot_d, x_dot_d);
phi_dot_d = gradient(phi_z_d, dt);

% ====== Convert to rover frame velocities ======
x_dot_body_d = zeros(1,N);
y_dot_body_d = zeros(1,N);

for k = 1:N
    % Rotation from global to body frame
    R_IB = [ cos(phi_z_d(k))  sin(phi_z_d(k));
            -sin(phi_z_d(k))  cos(phi_z_d(k)) ];
    v_global = [x_dot_d(k); y_dot_d(k)];
    v_body   = R_IB * v_global;

    x_dot_body_d(k) = v_body(1);
    y_dot_body_d(k) = v_body(2);
end

% Desired states in rover frame → use in IK
% desired = [x_dot_body_d(k), y_dot_body_d(k), phi_dot_d(k)];

x(1) = x_d(1); y(1) = y_d(1); phi_z(1) = phi_z_d(1);

% ================== LOGS FOR WHEELS ======================
psi_log   = zeros(6,N);   % steering angles [rad]
theta_log = zeros(6,N);   % wheel spin rates [rad/s]

% ================== SIMULATION =====================
for k = 1:N-1
    desired = [x_dot_d(k), y_dot_d(k), phi_dot_d(k)];
    sensed  = [0, 0];   % assume no pitch/roll for now

    % Inverse kinematics → steering, wheel rates
    [psi, theta_dot] = inverse_kinematics(desired, sensed, roverParams, dhParams, wheels);
    for i = 1:6, wheels(3,i) = psi(i); end

    % log wheel states
    psi_log(:,k)   = psi;        % steering [rad]
    theta_log(:,k) = theta_dot;  % spin rate [rad/s]
    
    % Build actuation vector [psi_dot; theta_dot]
    actuation = [zeros(1,6); theta_dot'];   % size 1x12
    
    % Forward kinematics → actual velocities
    un_dot = forward_kinematics([0;0], actuation, roverParams, dhParams, wheels, ones(6,1));

    R = [cos(phi_z(k)) sin(phi_z(k)) 0;
         -sin(phi_z(k)) cos(phi_z(k)) 0;
         0 0 1];

    un_dot_b = un_dot;
    un_dot_b(1:3) = R * un_dot(1:3);

    % Extract
    xdot(k)     = un_dot(1);
    ydot(k)     = un_dot(2);
    zdot(k)     = un_dot(3);
    phizdot(k)  = un_dot(4);

    % Integrate states
    x(k+1)     = x(k)     + xdot(k)*dt;
    y(k+1)     = y(k)     + ydot(k)*dt;
    z(k+1)     = z(k)     + zdot(k)*dt;
    phi_z(k+1) = phi_z(k) + phizdot(k)*dt;

    xdot(k)     = un_dot_b(1);
    ydot(k)     = un_dot_b(2);
    zdot(k)     = un_dot_b(3);
    phizdot(k)  = un_dot_b(4);
end

% ================== PLOTTING ======================
figure;
subplot(4,1,1); hold on; grid on;
plot(tvec, x_d, 'r--', 'LineWidth',1.5);
plot(tvec, x,   'b-', 'LineWidth',1.5);
ylabel('x [m]'); legend('Desired','Actual');

subplot(4,1,2); hold on; grid on;
plot(tvec, y_d, 'r--', 'LineWidth',1.5);
plot(tvec, y,   'b-', 'LineWidth',1.5);
ylabel('y [m]'); legend('Desired','Actual');

subplot(4,1,3); hold on; grid on;
plot(tvec, phi_z_d * 180/pi, 'r--', 'LineWidth',1.5);
plot(tvec, phi_z * 180/pi,   'b-', 'LineWidth',1.5);
ylabel('\phi_z [deg]'); legend('Desired','Actual');
xlabel('Time [s]');

subplot(4,1,4); hold on; grid on;
plot(x_d, y_d, 'r--', 'LineWidth',1.5);
plot(x, y,   'b-', 'LineWidth',1.5);
ylabel('y (m)'); legend('Desired','Actual');
xlabel('x (m)');


figure;
subplot(3,1,1); hold on; grid on;
plot(tvec, x_dot_body_d, 'r--'); plot(tvec, xdot, 'b-');
ylabel('ẋ [m/s]'); legend('Desired','Actual');

subplot(3,1,2); hold on; grid on;
plot(tvec, y_dot_body_d, 'r--'); plot(tvec, ydot, 'b-');
ylabel('ẏ [m/s]'); legend('Desired','Actual');

subplot(3,1,3); hold on; grid on;
plot(tvec, phi_dot_d, 'r--'); plot(tvec, phizdot, 'b-');
ylabel('\phi̇_z [rad/s]'); legend('Desired','Actual');
xlabel('Time [s]');
figure;
subplot(2,1,1); hold on; grid on;
plot(tvec, psi_log'*180/pi, 'LineWidth',1.2);
ylabel('\psi [deg]');
title('Steering Angles');
legend('Wheel1','Wheel2','Wheel3','Wheel4','Wheel5','Wheel6');

subplot(2,1,2); hold on; grid on;
plot(tvec, theta_log'*60/(2*pi), 'LineWidth',1.2); % convert rad/s → RPM
ylabel('\thetȧ [RPM]');
xlabel('Time [s]');
title('Wheel Spin Rates');
legend('Wheel1','Wheel2','Wheel3','Wheel4','Wheel5','Wheel6');

%% ====================================== INVERSE KINEMATICS ===============================================
function [psi_c, theta_dot_c] = inverse_kinematics(desired_states, sensed_states,...
                                                               roverParams, dhParams, wheels)

    x_dot_d = desired_states(1); y_dot_d = desired_states(2); phiz_dot_d = desired_states(3);
    phix_dot_s = sensed_states(1); phiy_dot_s = sensed_states(2);

    psi_c = zeros(6, 1);
    psi_dot_c = 0;
    theta_dot_c = zeros(6, 1);

    rho = [0, 0, dhParams(2), dhParams(3), dhParams(2), dhParams(3)];
    rho_dot = [0, 0, dhParams(5), dhParams(6), dhParams(5), dhParams(6)];
    a_D = roverParams(2);
    d_D = roverParams(1);
    d_S = roverParams(3);
    a_rho = roverParams(4);
    gamma = roverParams(5);
    S = diag([1, 1, 1, d_S^2, d_S^2, d_S^2]);

    for i = 1:6
        b = (-1)^i;
        % -------------------- steering commanded ----------------------
        if abs(phiz_dot_d) < 1e-6
            psi_c(i) = atan(y_dot_d / x_dot_d);
        else
            r_x = -y_dot_d / phiz_dot_d;
            r_y = x_dot_d / phiz_dot_d;
            
            delta = wheels(7,i);
            sigma1 = delta - b*dhParams(1) + rho(i);
            r_x1 = r_x*cos(sigma1) - wheels(2,i)*sin(delta) - a_D*cos(sigma1) + d_D*sin(sigma1) ...
                   - wheels(1,i)*cos(delta);
    
            if i ~= 1 && i ~= 2
                 r_x1 = r_x1 + a_rho*cos(delta + gamma + rho(i));
            end
    
            r_y1 = r_y + b*d_S;
        
            psi_c(i) = -atan(r_x1 / r_y1);
    
            % ----------------- steering rate commanded --------------------
            delta_dot = wheels(12,i);
            sigma2 = delta_dot - b*dhParams(4) + rho_dot(i);
            num = r_x*sin(sigma1)*sigma2 + wheels(2,i)*cos(delta)*delta_dot - a_D*sin(sigma1)*sigma2 ...
                - d_D*cos(sigma1)*sigma2 - wheels(1,i)*sin(delta)*delta_dot;
    
            if i ~= 1 && i ~= 2
                 num = num + a_rho*sin(delta + gamma + rho(i))*(delta_dot + rho_dot(i));
            end
    
            psi_dot_c = (num * r_y1) / (r_x1^2 + r_y1^2);
            % psi_dot_c = psi_c(i)/0.1;
        end

        % ----------------- wheel speed commanded ----------------------
        J_psi = psi_jacobian(roverParams, dhParams, wheels(:,i), i);

        X = x_dot_d + b*d_D*dhParams(4) - (d_D - a_rho*sin(gamma + b*dhParams(1)))*rho_dot(i) ...
            - J_psi(1)*psi_dot_c;
        Y = y_dot_d - J_psi(2)*psi_dot_c;
        Z = -b*a_D*dhParams(4) - (a_rho*cos(gamma + b*dhParams(1)) - a_D)*rho_dot(i)...
            - J_psi(3)*psi_dot_c;
        PHI_X = phix_dot_s - J_psi(4)*psi_dot_c;
        PHI_Y = phiy_dot_s - b*dhParams(4) + rho_dot(i);
        PHI_Z = phiz_dot_d - J_psi(6)*psi_dot_c;
        
        psi = wheels(3,i);
        wheels(3,i) = psi_c(i);

        J_theta = theta_jacobian(roverParams, dhParams, wheels(:,i), i);
        J_zeta = zeta_jacobian(roverParams, dhParams, wheels(:,i), i);
        J_delta = delta_jacobian(roverParams, dhParams, wheels(:,i), i);

        wheels(3,i) = psi;
    
        z_col = [0; 0; -1; 0; 0; 0];
        A = [J_theta, z_col, J_zeta, J_delta];

        chi = pinv(A'*S*A) * A' * S * [X; Y; Z; PHI_X; PHI_Y; PHI_Z];
        theta_dot_c(i) = chi(1);
    end
end

% ====================================== FORWARD KINEMATICS ===============================================
function un_dot = forward_kinematics(sensed_states, actuation, roverParams, dhParams, wheels, lambda)

    a_D = roverParams(2);
    d_D = roverParams(1);
    d_S = roverParams(3);
    a_rho = roverParams(4);
    gamma = roverParams(5);

    En = [eye(3,4); zeros([2,4]); [0 0 0 1]];
    Es = [zeros(3,2); eye(2); [0 0]];
    S = diag([1, 1, 1, d_S^2, d_S^2, d_S^2]);
    us_dot = sensed_states;
    ps_dot = [dhParams(4); dhParams(5); dhParams(6); actuation(:)];

    sum_lambda_G = zeros(4,6);
    sum_lambda_G_Jsq = zeros(4,3);
    lambda_G_Jsa_i = zeros(4,2*6);

    for i = 1:6
        b = (-1)^i;
        wheelParams = wheels(:,i);

        J_zeta = zeta_jacobian(roverParams, dhParams, wheelParams, i);
        % J_eta = eta_jacobian(dhParams, wheelParams, i);
        J_delta = delta_jacobian(roverParams, dhParams, wheelParams, i);
        
        Pn = [J_zeta J_delta];
        G = En' * (eye(6) - S*Pn*inv(Pn'*S*Pn)*Pn') * S;

        Js_q(:,1) = [-b*d_D; 0; b*a_D; 0; b; 0];
        if i == 1 || i == 2
            Js_q(:,2:3) = zeros(6,2);
        elseif mod(i,2) ~= 0
            Js_q(:,2) = [d_D-a_rho*sin(gamma+b*dhParams(1)); 0; a_rho*cos(gamma+b*dhParams(1))-a_D...
                           ;0; -1; 0];
            Js_q(:,3) = zeros(6,1);
        else 
            Js_q(:,3) = [d_D-a_rho*sin(gamma+b*dhParams(1)); 0; a_rho*cos(gamma+b*dhParams(1))-a_D...
                           ;0; -1; 0];
            Js_q(:,2) = zeros(6,1);
        end

        J_psi = psi_jacobian(roverParams, dhParams, wheelParams, i);
        J_theta = theta_jacobian(roverParams, dhParams, wheelParams, i);
        Js_a = [J_psi J_theta];

        sum_lambda_G = sum_lambda_G + lambda(i)*G;
        sum_lambda_G_Jsq = sum_lambda_G_Jsq + lambda(i)*G*Js_q;
        lambda_G_Jsa_i(:,2*(i-1)+1:2*i) = lambda(i)*G*Js_a;
    end

    un_dot = inv(sum_lambda_G*En) * [-sum_lambda_G*Es, sum_lambda_G_Jsq, lambda_G_Jsa_i] * [us_dot; ps_dot];
end

% =========================================== JACOBIANS ===================================================
% ========================================= psi ===========================================================
function J_psi = psi_jacobian(roverParams, dhParams, wheelParams, wheel_number)

    if wheel_number == 1 || wheel_number == 2
        rho = 0;
        a_rho = 0;
    elseif mod(wheel_number,2) ~= 0
        rho = dhParams(2);
        a_rho = roverParams(4);
    else 
        rho = dhParams(3);
        a_rho = roverParams(4);
    end

    b = (-1)^wheel_number;
    sigma = rho - b*dhParams(1);

    J_x = b*roverParams(3)*cos(sigma);
    J_y = wheelParams(1) + roverParams(2)*cos(sigma) - roverParams(1)*sin(sigma) ...
          - a_rho*cos(roverParams(5)+rho);
    J_z = -b*roverParams(3)*sin(sigma);
    J_phix = sin(sigma);
    J_phiz = cos(sigma);

    J_psi = [J_x; J_y; J_z; -J_phix; 0; -J_phiz];
end

% ========================================= theta =========================================================
function J_theta = theta_jacobian(roverParams, dhParams, wheelParams, wheel_number)

    if wheel_number == 1 || wheel_number == 2
        rho = 0;
    elseif mod(wheel_number,2) ~= 0
        rho = dhParams(2);
    else 
        rho = dhParams(3);
    end
    beta = dhParams(1);
    psi = wheelParams(3);
    delta = wheelParams(7);
    zeta = wheelParams(5);
    r = roverParams(6);

    b = (-1)^wheel_number;
    sigma = rho - b*beta;

    J_x = r * (sin(zeta)*sin(psi)*cos(sigma) - sin(delta)*cos(zeta)*sin(sigma) ...
          + cos(delta)*cos(psi)*cos(zeta)*cos(sigma));
    J_y = r * (-cos(psi)*sin(zeta) + cos(delta)*sin(psi)*cos(zeta));
    J_z = r * (-sin(delta)*cos(zeta)*cos(sigma) - sin(zeta)*sin(psi)*sin(sigma) ...
          - cos(delta)*cos(psi)*cos(zeta)*sin(sigma));

    J_theta = [J_x; J_y; J_z; 0; 0; 0];
end

% ========================================== zeta =========================================================
function J_zeta = zeta_jacobian(roverParams, dhParams, wheelParams, wheel_number)

    if wheel_number == 1 || wheel_number == 2
        rho = 0;
        a_rho = 0;
    elseif mod(wheel_number,2) ~= 0
        rho = dhParams(2);
        a_rho = roverParams(4);
    else 
        rho = dhParams(3);
        a_rho = roverParams(4);
    end
    beta = dhParams(1);
    gamma = roverParams(5);
    psi = wheelParams(3);
    delta = wheelParams(7);
    d_S = roverParams(3);
    a_S = wheelParams(1);
    d_D = roverParams(1);
    a_D = roverParams(2);

    b = (-1)^wheel_number;
    sigma = rho - b*beta;

    J_x = -b*d_S*cos(delta)*cos(sigma) + b*d_S*sin(delta)*cos(psi)*sin(sigma) ...
         + wheelParams(2)*sin(delta)*sin(psi)*cos(sigma) + a_rho*sin(psi)*sin(delta)*sin(gamma+b*beta) ...
         + a_S*sin(delta)*sin(psi)*sin(sigma) - d_D*sin(delta)*sin(psi);
    J_y = a_rho*cos(delta)*cos(gamma+rho) - a_rho*cos(psi)*sin(delta)*sin(gamma+rho) ...
         - a_D*cos(delta)*cos(sigma) + a_D*cos(psi)*sin(delta)*sin(sigma) ...
         + d_D*cos(delta)*sin(sigma) + d_D*cos(psi)*sin(delta)*cos(sigma) ...
         - a_S*cos(delta) - wheelParams(2)*cos(psi)*sin(delta);
    J_z = a_D*sin(delta)*sin(psi) + b*d_S*cos(delta)*sin(sigma) + b*d_S*cos(psi)*sin(delta)*cos(sigma) ...
         + a_S*sin(delta)*sin(psi)*cos(sigma) - a_rho*sin(delta)*sin(psi)*cos(gamma+b*beta) ...
         - wheelParams(2)*sin(delta)*sin(psi)*sin(sigma);
    J_phix = -cos(psi)*sin(delta)*cos(sigma) - cos(delta)*sin(sigma);
    J_phiy = -sin(delta)*sin(psi);
    J_phiz = cos(psi)*sin(delta)*sin(sigma) - cos(delta)*cos(sigma);

    J_zeta = [J_x; J_y; J_z; -J_phix; -J_phiy; -J_phiz];
end

% =========================================== eta =========================================================
function J_eta = eta_jacobian(dhParams, wheelParams, wheel_number)

    if wheel_number == 1 || wheel_number == 2
        rho = 0;
    elseif mod(wheel_number,2) ~= 0
        rho = dhParams(2);
    else 
        rho = dhParams(3);
    end
    beta = dhParams(1);
    psi = wheelParams(3);
    delta = wheelParams(7);
    zeta = wheelParams(5);

    b = (-1)^wheel_number;
    sigma = rho - b*beta;

    J_x = cos(zeta)*sin(psi)*cos(sigma) + sin(delta)*sin(zeta)*sin(sigma) ...
          - cos(delta)*cos(psi)*cos(zeta)*cos(sigma);
    J_y = -cos(psi)*cos(zeta) - cos(delta)*sin(psi)*sin(zeta);
    J_z = sin(delta)*sin(zeta)*cos(sigma) - cos(zeta)*sin(psi)*sin(sigma) ...
          + cos(delta)*cos(psi)*sin(zeta)*sin(sigma);

    J_eta = [J_x; J_y; J_z; 0; 0; 0];
end

% ========================================= delta =========================================================
function J_delta = delta_jacobian(roverParams, dhParams, wheelParams, wheel_number)

    if wheel_number == 1 || wheel_number == 2
        rho = 0;
        a_rho = 0;
    elseif mod(wheel_number,2) ~= 0
        rho = dhParams(2);
        a_rho = roverParams(4);
    else 
        rho = dhParams(3);
        a_rho = roverParams(4);
    end
    beta = dhParams(1);
    gamma = roverParams(5);
    psi = wheelParams(3);
    d_S = roverParams(3);
    a_S = wheelParams(1);
    d_D = roverParams(1);
    a_D = roverParams(2);

    b = (-1)^wheel_number;
    sigma = rho - b*beta;

    J_x = d_D*cos(psi) - a_rho*cos(psi)*sin(gamma+b*beta) - a_S*cos(psi)*sin(sigma) ...
         - wheelParams(2)*cos(psi)*cos(sigma) + b*d_S*sin(psi)*sin(sigma);
    J_y = -sin(psi) * (wheelParams(2) - d_D*cos(sigma) - a_D*sin(sigma) + a_rho*sin(gamma+rho));
    J_z = -a_D*cos(psi) + a_rho*cos(psi)*cos(gamma+b*beta) - a_S*cos(psi)*cos(sigma) ...
         + wheelParams(2)*cos(psi)*sin(sigma) + b*d_S*sin(psi)*cos(sigma);
    J_phix = -sin(psi) * cos(sigma);
    J_phiy = cos(psi);
    J_phiz = sin(psi) * sin(sigma);

    J_delta = [J_x; J_y; J_z; -J_phix; -J_phiy; -J_phiz];
end
