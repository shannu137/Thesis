function ekf = run_rover_ekf(t, inputs, measurements, Sigma_VO, x0, dhParams, params)
    N = length(t);
    nx = 27;
    
    x = zeros(nx,1);
    
    x(1:3) = x0(1:3);
    x(4:6) = x0(4:6);
    x(7:9) = x0(7:9);
    x(10:12) = params.imu.Gyroscope.ConstantBias'; %zeros(3,1);
    x(13:15) = params.imu.Accelerometer.ConstantBias'; %zeros(3,1);
    x(16:21) = zeros(6,1);
    x(22:27) = zeros(6,1);
    
    P = params.P0;
    
    ekf.x = zeros(nx,N);
    ekf.P = cell(N,1);
    
    ekf.x(:,1) = x;
    ekf.P{1} = P;

    for k = 2:N
        dt = t(k) - t(k-1);

        u = inputs(:,k);
        [x_pred, F, G] = ekf_propagation(x, u, dt, dhParams(k), params);
    
        Q = params.Q;
        P_pred = F * P * F' + G * Q * G' * dt;
      
        z = measurements(:,k);
        R = blkdiag(Sigma_VO{k}, params.noise_enc^2*eye(6), params.noise_curr^2*eye(6));
        [x, P] = ekf_vo_update(x_pred, P_pred, z, R);
    
        ekf.x(:,k) = x;
        ekf.P{k} = P;
    end
end

function [xnew, F, G] = ekf_propagation(x, inputs, dt, dhParams, params)
    p = x(1:3);
    v = x(4:6);
    ang = x(7:9);
    bg = x(10:12);
    ba = x(13:15);
    Omega = x(16:21);
    I = x(22:27);

    wm = inputs(1:3);
    am = inputs(4:6);
    Vin = inputs(7:12);
    
    R = singleAxisDCM(1,ang(1)) * singleAxisDCM(2,ang(2)) * singleAxisDCM(3,ang(3));
    B = euler_rate_matrix(ang);
    
    wc = wm - bg;
    ac = am - ba;
    
    g = [0;0;-params.gravity];
    
    % Fres = params.c_rr * ones(6,1);
    
    pdot = v;
    vdot = -R'*ac + g;
    thetadot = B * wc;
    bgdot = zeros(3,1);
    badot = zeros(3,1);

    Ft = zeros(6,1);
    Fts = zeros(6,1);
    [vt, Jt] = compute_tangential_velocity(x, wm, dhParams,  params);
    for i = 1:6
        [Ft(i), Fts(i)] = bekker_force(x(15+i), vt(i), params);
    end

    Omegadot = zeros(6,1);
    for i = 1:6
        Omegadot(i) = (1/params.motor_J) * (params.motor_Kt*I(i) - params.motor_b*Omega(i) - params.wheel_radius*Ft(i)); % + params.r_wheel * Fres(i));
    end

    Idot = zeros(6,1);
    for i = 1:6
        Idot(i) = (1/params.motor_L) * (Vin(i) - params.motor_R*I(i) - params.motor_Ke*Omega(i));
    end
    
    xnew = x;
    
    xnew(1:3) = p + dt*pdot;
    xnew(4:6) = v + dt*vdot;
    xnew(7:9) = ang + dt*thetadot;
    xnew(10:12) = bg + dt*bgdot;
    xnew(13:15) = ba + dt*badot;
    xnew(16:21) = Omega + dt*Omegadot;
    xnew(22:27) = I + dt*Idot;

    [F, G] = compute_state_jacobians(x, am, wm, vt, Jt, Fts, params, dt);
end

function [xupd, Pupd] = ekf_vo_update(xpred, Ppred, z, R)

    p = xpred(1:3);
    ang = xpred(7:9);
    Omega = xpred(16:21);
    I = xpred(22:27);

    h = [p; ang; Omega; I];
    
    H = zeros(18,27);
    
    H(1:3,1:3) = eye(3);
    H(4:6,7:9) = eye(3);
    H(7:12,16:21) = eye(6);
    H(13:18,22:27) = eye(6);
    
    r = z - h;
    S = H * Ppred * H' + R;
    K = Ppred * H' / S;
    xupd = xpred + K * r;
    
    I27 = eye(27);
    Pupd = (I27 - K*H)*Ppred*(I27 - K*H)' + K*R*K';
end

function [F, G] = compute_state_jacobians(x, am, wm, vt, Jt, Fts, params, dt)
    
    nx = 27;
    nw = 24;

    F = eye(nx);  
    G = zeros(nx, nw);
    
    ang = x(7:9);
    Omega = x(16:21);
    phi = ang(1);
    theta = ang(2);
    psi = ang(3);
    
    R = singleAxisDCM(1,phi) * singleAxisDCM(2,theta) * singleAxisDCM(3,psi);
    
    F(1:3,4:6) = dt * eye(3);
    
    ac = am - x(13:15);
    F(4:6,7:9) = dt * R' * tilde(ac);    
    F(4:6,13:15) = dt * R';
    
    wc = wm - x(10:12);
    B = euler_rate_matrix(ang);
    F(7:9,10:12) = -dt * B;
    dBomega = [(wc(2)*cos(phi)-wc(3)*sin(phi))*tan(theta), (wc(2)*sin(phi)+wc(3)*cos(phi))/(cos(theta))^2, 0;
                -(wc(2)*sin(phi)+wc(3)*cos(phi)), 0, 0;
               (wc(2)*cos(phi)-wc(3)*sin(phi))/cos(theta), (wc(2)*sin(phi)+wc(3)*cos(phi))*sin(theta)/(cos(theta))^2, 0];
    F(7:9, 7:9) = eye(3) + dBomega * dt;

    for i = 1:6
        row = 15 + i;
        if Omega(i) == 0
            continue;
        end
        Om = sign(Omega(i)) * max(abs(Omega(i)),1e-3);
        F(row, 4:6) = dt * Fts(i) * Jt(:,i)' / (params.motor_J * Om);
        F(row,row) = 1 - dt*params.motor_b/params.motor_J - dt*Fts(i)*vt(i)/(params.motor_J*Om^2);
        F(row,21+i) = dt*params.motor_Kt/params.motor_J;
    end

    for i = 1:6
        row = 21 + i;
        F(row,row) = 1 - dt*params.motor_R/params.motor_L;
        F(row,15+i) = -dt*params.motor_Ke/params.motor_L;
    end

    G(4:6, 4:6) = -R';
    G(7:9, 1:3) = -B;
    G(10:27, 7:24) = eye(18);
end

function B = euler_rate_matrix(eul)
    phi = eul(1);
    theta = eul(2);
    
    B = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
end

function X = tilde(x)
    X = [0 -x(3) x(2);
         x(3) 0 -x(1);
         -x(2) x(1) 0];
end

function [Ft, dFtds] = bekker_force(Omega, vt, params)

    r = params.wheel_radius;
    b = params.wheel_width;
    kc   = params.soil_kc;
    kphi = params.soil_kphi;
    n    = params.soil_n;
    c    = params.soil_c;
    phi  = params.soil_phi;
    K    = params.soil_K;
    
    Ww = params.rover_mass * params.gravity / 6;
    keff = kc + b*kphi;
    z0 = (Ww^2 * b^(2*(n-1)) / (keff^2* 2 * r))^(1/(2*n+1));
    
    theta1 = acos(1 - z0/r);
    theta2 = -0.4 * theta1;
    
    [xi,w] = gauss_legendre_20();
    
    alpha = (theta1-theta2)/2;
    beta  = (theta1+theta2)/2;
    
    theta = alpha*xi + beta;
    wq    = alpha*w;
    
    ct = cos(theta);
    st = sin(theta);
    
    zth = r*(ct - cos(theta1));
    sigma = keff * (zth/b).^n;

    s = 1 - vt ./ (r*Omega + 1e-12);
    
    j = r*((theta1-theta) -(1-s)*(sin(theta1)-st));
    j = max(j,0);
    
    tau = (c + sigma*tan(phi)) .* (1 - exp(-j/K));

    Ft = r*b*sum(wq .* (tau.*ct - sigma.*st));
    
    djds = r*(sin(theta1)-st);
    dtauds = (c + sigma*tan(phi)) .* exp(-j/K) .* (djds/K);
    dFtds = r*b*sum(wq .* (dtauds .* ct));
end

function [xi, w] = gauss_legendre_20()

    xi = [ ...
        -0.993128599185094859; -0.963971927277913791; ...
        -0.912234428251325905; -0.839116971822218823; ...
        -0.746331906460150793; -0.636053680726515024; ...
        -0.510867001950827097; -0.373706088715419561; ...
        -0.227785851141645078; -0.076526521133497333; ...
         0.076526521133497333;  0.227785851141645078; ...
         0.373706088715419561;  0.510867001950827097; ...
         0.636053680726515024;  0.746331906460150793; ...
         0.839116971822218823;  0.912234428251325905; ...
         0.963971927277913791;  0.993128599185094859];

    w = [ ...
        0.017614007139152118; 0.040601429800386941; ...
        0.062672048334109064; 0.083276741576704748; ...
        0.101930119817240435; 0.118194531961518417; ...
        0.131688637831675917; 0.142096109318382051; ...
        0.149172986472603747; 0.152753387130725850; ...
        0.152753387130725850; 0.149172986472603747; ...
        0.142096109318382051; 0.131688637831675917; ...
        0.118194531961518417; 0.101930119817240435; ...
        0.083276741576704748; 0.062672048334109064; ...
        0.040601429800386941; 0.017614007139152118];

end