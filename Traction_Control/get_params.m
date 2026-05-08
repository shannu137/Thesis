function params = get_params()
    params.fs     = 100;           % Sampling rate [Hz]
    params.Ttotal = 60;            % Total simiulation time [s]
    
    params.terrain_Lx      = 4;    % Terrain size X [m]
    params.terrain_Ly      = 3;    % Terrain size Y [m]
    params.terrain_res_oct = 8;    % As number of octaves (res = 2^N + 1)
    
    params.terrain_numHills   = 10;
    params.terrain_numCraters = 10;

    params.rover_gnd_clr      = 0.3;  % rover ground clearance [m]
    params.rover_length       = 0.55; % rover length [m]
    params.rover_width        = 0.65; % rover width  [m]
    params.rover_height       = 0.3;  % rover height [m]
    params.rover_slope_limit  = deg2rad(25);
    params.rover_rough_limit  = 0.1;
    params.rover_vel_limit    = 0.1;  % m/s
    params.rover_acc_limit    = 0.05; % m/s^2
    
    params.rover_d_D          = 0;
    params.rover_a_D          = 0;
    params.rover_d_S          = (18.562 + 13.176) * 1e-02;
    params.rover_a_rho        = sqrt(12.454^2 + 3.3492^2) * 1e-02;
    params.rover_gamma        = atan2(3.3492, 12.454);
    params.wheel_radius       = 6e-2;
    params.wheel_width        = 3e-2;
    
    params.rover_a_S_vals = [25.1, 25.1, 12.568, 12.568, -12.499, -12.499]*1e-2;
    params.rover_d_W_vals = [(10.408+8.1978), (10.408+8.1978), (7.0577+8.1978), ...
                             (7.0577+8.1978), (7.0577+8.1978), (7.0577+8.1978)]*1e-2;
    
    params.cost_w_distance  = 1;
    params.cost_w_turn      = 0.1;
    params.cost_w_slope     = 50;
    params.cost_w_rough     = 10;

    params.N_path_samples = 500;  % #samples for s-parametrization in build_path

    params.ema_alpha     = 0.1;
    params.ema_alpha_der = 0.05;

    params.gravity    = 1.625; % m/s^2
    params.rover_mass = 15;    % kg

    params.soil_kc   = 0.14e4;      % [Pa]
    params.soil_kphi = 0.82e6;      % [Pa/m]
    params.soil_n    = 1.0;
    params.soil_c    = 0.017e4;     % [Pa]
    params.soil_phi  = deg2rad(35); % [rad]
    params.soil_K    = 1.78e-2;     % [m]

    % d2r = pi / 180;
    % g = params.gravity;
    % 
    % params.IMU = imuSensor('accel-gyro', ...
    %             'Accelerometer', accelparams('MeasurementRange', 2 * g, ...       % +-2g
    %                                          'Resolution', 1e-03 * g, ...         % 1 mg/LSB
    %                                          'NoiseDensity', 220e-06 * g, ...     % 220 µg/sqrt(Hz)
    %                                          'ConstantBias', 60e-03 * g, ...      % +-60 mg
    %                                          'TemperatureBias', 0.5e-03 * g), ... % +-0.5 mg/deg-C                 
    %             'Gyroscope', gyroparams('MeasurementRange', 250 * d2r, ...        % +-250 dps
    %                                     'Resolution', 8.75e-03 * d2r, ...         % 8.75 mdps/digit
    %                                     'NoiseDensity', 0.03 * d2r, ...           % 0.03 dps/sqrt(Hz)
    %                                     'ConstantBias', 10 * d2r, ...             % +-10 dps (typical)
    %                                     'TemperatureBias', 0.03 * d2r), ...       % +-0.03 dps/deg-C
    %             'ReferenceFrame', 'ENU'); 

    params.motorctrl_Kff  = 75;
    params.motorctrl_Kp   = 35;
    params.motorctrl_Ki   = 15;
    params.motor_V_supply = 12; % [V]

    params.motor_L = 0.2; % [H]
    params.motor_J = 0.1; % []
    params.motor_Kt = 2.9050;
    params.motor_Ke = 2.9050;
    params.motor_R  = 4.3576;
    params.motor_b  = 0.3783;

    params.encoder_CPR = 28080;
    
    params.noise_enc  = 0.01;
    params.noise_curr = 0.01;
end
