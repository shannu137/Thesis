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
    params.rover_acc_limit    = 0.05;  % m/s^2
    
    params.rover_d_D          = 0;
    params.rover_a_D          = 0;
    params.rover_d_S          = (18.562 + 13.176) * 1e-02;
    params.rover_a_rho        = sqrt(12.454^2 + 3.3492^2) * 1e-02;
    params.rover_gamma        = atan2(3.3492, 12.454);
    params.wheel_radius       = 6e-2;
    
    params.rover_a_S_vals = [25.1, 25.1, 12.568, 12.568, -12.499, -12.499]*1e-2;
    params.rover_d_W_vals = [(10.408+8.1978), (10.408+8.1978), (7.0577+8.1978), ...
                             (7.0577+8.1978), (7.0577+8.1978), (7.0577+8.1978)]*1e-2;
    
    params.cost_w_distance  = 1;
    params.cost_w_turn      = 0.1;
    params.cost_w_slope     = 50;
    params.cost_w_rough     = 10;
end
