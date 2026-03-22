params.fs     = 100;           % Sampling rate [Hz]
params.Ttotal = 60;            % Total simiulation time [s]

params.terrain_Lx      = 4;    % Terrain size X [m]
params.terrain_Ly      = 3;    % Terrain size Y [m]
params.terrain_res_oct = 8;    % As number of octaves (res = 2^N + 1)

params.terrain_numHills   = 10;
params.terrain_numCraters = 10;

params.rover_gnd_clr   = 0.15; % rover ground clearance [m]
params.rover_length    = 0.55;  % rover length [m]
params.rover_width     = 0.65; % rover width  [m]
params.rover_height    = 0.3;  % rover height [m]

params.cost_w_slope  = 0.5;
params.cost_w_depth  = 0.5;
params.cost_w_height = 0.5;