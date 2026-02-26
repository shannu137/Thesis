function generateDataset(parameters, cameraParams, outputFilename, showPlots)

    if(nargin < 4)
        showPlots = false;
    end

    numTrajectories = parameters.numTrajectories;
    N = parameters.numSamples;
    samplingRate = parameters.samplingRate;
    % numFeatures = 4;
    
    fprintf('Generating %d trajectories...\n', numTrajectories);

    % m = 29 + numFeatures*3;
    % n = 8*numFeatures + 3;

    x_IC = zeros(numTrajectories, m, 1);
    x_true = zeros(numTrajectories, m, N);
    u = zeros(numTrajectories, 6, N);
    y = zeros(numTrajectories, n, N);

    
    for trajID = 1:numTrajectories
        parameters.duration = N / samplingRate; 
    
        traj = generateTrajectory(parameters);
        if showPlots
            visualizeTrajectory(traj);
        end
        
        [accelReadings, gyroReadings, magReadings, bias] = simulateIMU(traj);
        if showPlots
            visualizeIMUmeasurements(traj.t, accelReadings, gyroReadings, magReadings)
        end
        
        [markers_global_all, pixel_coords_left_all, pixel_coords_right_all] = simulateCamera(traj, cameraParams);
        if showPlots
            visualizeCameraData(pixel_coords_left_all, cameraParams);
            visualizeCameraData(pixel_coords_right_all, cameraParams);
        end
        
        % True state vector [xk+1; xk; vk+1; vk; qk+1; qk; b_gk+1; b_ak+1; b_mk+1; P_i]
        p_true = traj.pos;
        v_true = traj.vel;
        q_true = traj.quat;
        b_m_true = repmat(bias.magnetometer, [N+1,1])' + parameters.noiseStd_b * randn(3,N+1);
        b_g_true = repmat(bias.gyroscope, [N+1,1])' + parameters.noiseStd_b * randn(3,N+1);
        b_a_true = repmat(bias.accelerometer, [N+1,1])' + parameters.noiseStd_b * randn(3,N+1);
        markers_global = reshape(cat(3, markers_global_all{:}), [12,N+1]);

        zerovec = zeros(3,1);
        x_IC(trajID,:,:) = [p_true(:,1); zerovec; v_true(:,1); zerovec; q_true(:,1); 1; zerovec; b_g_true(:,1); ...
                       b_a_true(:,1); b_m_true(:,1); zeros(numFeatures*3,1)];

        % from t = 1 to N
        x_true(trajID,:,:) = [p_true(:,2:end); p_true(:,1:end-1); ...
                 v_true(:,2:end); v_true(:,1:end-1); ...
                 q_true(:,2:end); q_true(:,1:end-1); ...
                 b_g_true(:,2:end); b_a_true(:,2:end); b_m_true(:,2:end); ...
                 markers_global(:,1:end-1)];
        
        % Control input (gyro and accelerometer) from t=0 to N-1
        u(trajID,:,:) = [gyroReadings(1:N,:)'; accelReadings(1:N,:)'];
        
        % Measurements ([(uL_iPrev; uR_iPrev; uL_iCurr; uR_iCurr); ... ;
        % mag]) from t = 1 to N
        z = zeros(n, N);
        
        for k = 1:N
            for i = 1:numFeatures
                z(8*(i-1)+1:8*(i-1)+2, k) = pixel_coords_left_all{k}(:,i);
                z(8*(i-1)+3:8*(i-1)+4, k) = pixel_coords_right_all{k}(:,i);
                z(8*(i-1)+5:8*(i-1)+6, k) = pixel_coords_left_all{k+1}(:,i);
                z(8*(i-1)+7:8*(i-1)+8, k) = pixel_coords_right_all{k+1}(:,i);
            end
            z(end-2:end, k) = magReadings(k+1,:)';
        end

        y(trajID,:,:) = z;
    
        if mod(trajID, 10) == 0
            fprintf('Generated %d/%d trajectories\n', trajID, numTrajectories);
        end
    end
    
    dataset = struct();
    dataset.x_IC = x_IC;
    dataset.x_true = x_true;
    dataset.u = u;
    dataset.y = y;

    fprintf('Dataset generation complete!\n');
    save(outputFilename, 'x_IC', 'x_true', 'u', 'y');
end

% ------------------------ Functions ------------------------
function visualizeTrajectory(trajectory)

    figure('Name', 'Trajectory Visualization', 'NumberTitle', 'off');
    subplot(3,2,1);
    plot(trajectory.t, trajectory.pos, 'LineWidth',1.5);
    xlabel('Time [s]'); ylabel('Position [m]');
    legend('x','y','z'); grid on;
    title('Position vs Time');

    subplot(3,2,2);
    plot(trajectory.t, trajectory.vel, 'LineWidth',1.5);
    xlabel('Time [s]'); ylabel('Velocity [m/s]');
    legend('v_x','v_y','v_z'); grid on;
    title('Velocity vs Time');

    subplot(3,2,3);
    plot(trajectory.t, trajectory.acc, 'LineWidth',1.5); 
    xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
    legend('a_x','a_y','a_z'); grid on;
    title('Acceleration vs Time');

    subplot(3,2,4);
    plot(trajectory.t, rad2deg(trajectory.eul), 'LineWidth',1.5);
    xlabel('Time [s]'); ylabel('Euler angles [deg]');
    legend('Yaw','Pitch','Roll'); grid on;
    title('Orientation (body intrinsic ZYX)');

    subplot(3,2,5);
    plot(trajectory.t, trajectory.angvel_global, 'LineWidth',1.5);
    xlabel('Time [s]'); ylabel('Angular velocity [rad/s]');
    legend('\omega_x','\omega_y','\omega_z');
    grid on; title('Angular Velocity (global frame)');
    
    subplot(3,2,6);
    plot(trajectory.t, trajectory.angvel, 'LineWidth',1.5);
    xlabel('Time [s]'); ylabel('Angular velocity [rad/s]');
    legend('\omega_x','\omega_y','\omega_z'); grid on;
    title('Angular Velocity (body frame)');

    sgtitle('Trajectory State Evolution');

    figure('Name', '3D Trajectory', 'NumberTitle', 'off');
    plot3(trajectory.pos(1,:), trajectory.pos(2,:), trajectory.pos(3,:), 'LineWidth',2);
    hold on; grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('3D Position Trajectory');
end

function [accelReadings, gyroReadings, magReadings, bias] = simulateIMU(trajectory)

    g = 9.81;
    d2r = pi/180;
    gauss2utesla = 100;

    acc = trajectory.acc;
    angVel = trajectory.angvel_global;
    orientation = eul2rotmat(trajectory.eul, 'zyx');
    % orientation = permute(orientation, [2 1 3]);

    IMU = imuSensor('accel-gyro-mag', ...
        'Accelerometer', accelparams('MeasurementRange', 2 * g, ...       % +-2g
                                     'Resolution', 1e-03 * g, ...         % 1 mg/LSB
                                     'NoiseDensity', 220e-06 * g, ...     % 220 µg/sqrt(Hz)
                                     'ConstantBias', 60e-03 * g, ...        % +-60 mg
                                     'TemperatureBias', 0.5e-03 * g), ...  % +-0.5 mg/deg-C                 
        'Gyroscope', gyroparams('MeasurementRange', 250 * d2r, ...          % +-250 dps
                                'Resolution', 8.75e-03 * d2r, ...            % 8.75 mdps/digit
                                'NoiseDensity', 0.03 * d2r, ...             % 0.03 dps/sqrt(Hz)
                                'ConstantBias', 10 * d2r, ...               % +-10 dps (typical)
                                'TemperatureBias', 0.03 * d2r), ...         % +-0.03 dps/deg-C
        'Magnetometer', magparams('MeasurementRange', 1.3 * gauss2utesla, ...        % +-1.3 gauss
                                  'Resolution', 2e-03 * gauss2utesla),...           % 2 mgauss
        'ReferenceFrame', 'ENU'); 

    bias.accelerometer = IMU.Accelerometer.ConstantBias;
    bias.gyroscope = IMU.Gyroscope.ConstantBias;
    bias.magnetometer = IMU.Magnetometer.ConstantBias;

    [accelReadings, gyroReadings, magReadings] = IMU(acc', angVel', orientation);
end

function visualizeIMUmeasurements(t, accelReadings, gyroReadings, magReadings)
    figure;
    sgtitle('Accelerometer Readings')
    subplot(3,1,1); plot(t, accelReadings(:, 1), LineWidth=1.5); title('a_x'); ylabel('m/s^2');
    subplot(3,1,2); plot(t, accelReadings(:, 2), LineWidth=1.5); title('a_y'); ylabel('m/s^2');
    subplot(3,1,3); plot(t, accelReadings(:, 3), LineWidth=1.5); title('a_z'); ylabel('m/s^2'); xlabel('Time (s)');
    
    figure;
    sgtitle('Gyroscope Readings')
    subplot(3,1,1); plot(t, gyroReadings(:, 1), LineWidth=1.5); title('g_x'); ylabel('rad/s');
    subplot(3,1,2); plot(t, gyroReadings(:, 2), LineWidth=1.5); title('g_y'); ylabel('rad/s');
    subplot(3,1,3); plot(t, gyroReadings(:, 3), LineWidth=1.5); title('g_z'); ylabel('rad/s'); xlabel('Time (s)');
    
    figure;
    sgtitle('Magnetometer Readings')
    subplot(3,1,1); plot(t, magReadings(:, 1), LineWidth=1.5); title('m_x'); ylabel('\mu T');
    subplot(3,1,2); plot(t, magReadings(:, 2), LineWidth=1.5); title('m_y'); ylabel('\mu T');
    subplot(3,1,3); plot(t, magReadings(:, 3), LineWidth=1.5); title('m_z'); ylabel('\mu T'); xlabel('Time (s)');
end

function [markers_global_all, pixel_coords_left_all, pixel_coords_right_all] = simulateCamera(trajectory, cameraParams)
    
    baseline = cameraParams.baseline;  % m
    
    L = 0.05; W = 0.02; H = 0.10;
    local_markers = [-L/2,  L/2, -L/2,  L/2; 
                      W/2,  W/2,  W/2,  W/2; 
                     -H/2, -H/2,  H/2,  H/2];
    num_markers = size(local_markers, 2);
    
    R_C2G = cameraParams.R_C2G;

    camera_pos_left = cameraParams.t_G2CinG;
    camera_pos_right = camera_pos_left + R_C2G * [baseline; 0; 0];
    
    N = length(trajectory.t);
    markers_global_all = cell(N, 1);
    pixel_coords_left_all = cell(N, 1);
    pixel_coords_right_all = cell(N, 1);
    
    for i = 1:N
        pos = trajectory.pos(:, i);
        eul = trajectory.eul(:, i);
        roll = eul(3); pitch = eul(2); yaw = eul(1);
        
        Rz = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
        Ry = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
        Rx = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
        R_G2B = Rx * Ry * Rz;
        
        markers_global = R_G2B' * local_markers + pos;
        markers_left_frame = R_C2G' * (markers_global - camera_pos_left);
        markers_right_frame = R_C2G' * (markers_global - camera_pos_right);
        
        pixel_coords_left = projectToCamera(markers_left_frame, num_markers, cameraParams);
        pixel_coords_right = projectToCamera(markers_right_frame, num_markers, cameraParams);
        
        markers_global_all{i} = markers_global;
        pixel_coords_left_all{i} = pixel_coords_left;
        pixel_coords_right_all{i} = pixel_coords_right;
    end
end

function pixel_coords = projectToCamera(markers_camera_frame, num_markers, cameraParams)
    % Camera parameters
    image_width = cameraParams.image_width;
    image_height = cameraParams.image_height;
    focal_length = cameraParams.focal_length;  % pixels
    cx = cameraParams.cx;   % Principal point x
    cy = cameraParams.cy;  % Principal point y
    
    % Lens distortion coefficients
    k1 = cameraParams.k1;  % Radial distortion
    k2 = cameraParams.k2;
    p1 = cameraParams.p1;  % Tangential distortion
    p2 = cameraParams.p2;
    
    pixel_noise_std = cameraParams.pixel_noise_std;  % pixels
    enableQuantization = cameraParams.enableQuantization;

    pixel_coords = zeros(2, num_markers);
    
    for m = 1:num_markers
        Xc = markers_camera_frame(1, m);
        Yc = markers_camera_frame(2, m);
        Zc = markers_camera_frame(3, m);
        
        if Zc <= 0.01  % Too close or behind camera
            pixel_coords(:, m) = [NaN; NaN];
            continue;
        end
        
        x_norm = Xc / Zc;
        y_norm = Yc / Zc;
        
        r2 = x_norm^2 + y_norm^2;
        radial = 1 + k1*r2 + k2*r2^2;
        tangential_x = 2*p1*x_norm*y_norm + ...
                      p2*(r2 + 2*x_norm^2);
        tangential_y = p1*(r2 + 2*y_norm^2) + ...
                      2*p2*x_norm*y_norm;
        
        x_dist = x_norm * radial + tangential_x;
        y_dist = y_norm * radial + tangential_y;
        
        u_ideal = focal_length * x_dist + cx;
        v_ideal = focal_length * y_dist + cy;
        
        pixel_noise = pixel_noise_std * randn(2, 1);
        u_noisy = u_ideal + pixel_noise(1);
        v_noisy = v_ideal + pixel_noise(2);
        
        if enableQuantization
            u_quantized = round(u_noisy);
            v_quantized = round(v_noisy);
        else
            u_quantized = u_noisy;
            v_quantized = v_noisy;
        end

        if u_quantized < 1 || u_quantized > image_width || ...
           v_quantized < 1 || v_quantized > image_height
            pixel_coords(:, m) = [NaN; NaN];
            continue;
        end
        
        pixel_coords(:, m) = [u_quantized; v_quantized];
    end
end

function visualizeCameraData(pixel_coords_all, camera_params)
    N = length(pixel_coords_all);
    num_markers = size(pixel_coords_all{1}, 2);
    
    figure('Name', 'Camera Pixel Tracking', 'Position', [100 100 1200 600]);
    hold on; grid on;
    colors = lines(num_markers);
    
    for m = 1:num_markers
        px_traj = zeros(2, N);
        for i = 1:N
            if ~isnan(pixel_coords_all{i}(1, m))
                px_traj(:, i) = pixel_coords_all{i}(:, m);
            else
                px_traj(:, i) = [NaN; NaN];
            end
        end
        plot(px_traj(1, :), px_traj(2, :), '.', 'Color', colors(m, :), 'MarkerSize', 8);
    end
    
    xlabel('u [pixels]'); ylabel('v [pixels]');
    title('Marker Pixel Trajectories');
    xlim([0 camera_params.image_width]);
    ylim([0 camera_params.image_height]);
    set(gca, 'YDir', 'reverse');
    legend(arrayfun(@(x) sprintf('Marker %d', x), 1:num_markers, 'UniformOutput', false));
end

function [R_cam_meas_all, t_cam_meas_all] = processingCameraMeasurements(pixel_coords_left_all, pixel_coords_right_all, cameraParams)

    N = length(pixel_coords_left_all);
    R_cam_meas_all = cell(N, 1);
    t_cam_meas_all = cell(N, 1);
    
    R_cam_meas_all{1} = eye(3);
    t_cam_meas_all{1} = [0; 0; 0];

    R_C2G = cameraParams.R_C2G;

    camera_pos_from_global = cameraParams.t_G2CinG;

    points_3D_prev = triangulatePoints(pixel_coords_left_all{1}, ...
                                       pixel_coords_right_all{1}, ...
                                       cameraParams, cameraParams);
    
    for i = 2:N
        points_3D_curr = triangulatePoints(pixel_coords_left_all{i}, ...
                                           pixel_coords_right_all{i}, ...
                                           cameraParams, cameraParams);
        
        valid_prev = ~isnan(points_3D_prev(1, :));
        valid_curr = ~isnan(points_3D_curr(1, :));
        valid_both = valid_prev & valid_curr;
        
        if sum(valid_both) >= 4
            P_prev = points_3D_prev(:, valid_both);
            P_curr = points_3D_curr(:, valid_both);

            P_prev_G = R_C2G * P_prev + camera_pos_from_global;
            P_curr_G = R_C2G * P_curr + camera_pos_from_global;

            % P_curr_G = R_est * P_prev_G + t_est
            % R_Bk2Bk-1 is R_est and 
            % t_G2Bk_inG is given by substituting t_G2Bk-1_inG in 1st eqn
            [R_est, t_est] = calculateTransformation(P_curr_G, P_prev_G);
            
            R_cam_meas_all{i} = R_est' * R_cam_meas_all{i-1};
            t_cam_meas_all{i} = R_est * t_cam_meas_all{i-1} + t_est;
        else
            fprintf('Warning: Frame %d has only %d valid correspondences. Using identity.\n', ...
                    i, sum(valid_both));
            R_cam_meas_all{i} = R_cam_meas_all{i-1};
            t_cam_meas_all{i} = t_cam_meas_all{i-1};
        end
        
        points_3D_prev = points_3D_curr;
    end
end

function points_3D = triangulatePoints(pixel_coords_left, pixel_coords_right, cameraParams_l, cameraParams_r)

    num_points = size(pixel_coords_left, 2);
    points_3D = zeros(3, num_points);
    
    f_l = cameraParams_l.focal_length;
    cx_l = cameraParams_l.cx;
    cy_l = cameraParams_l.cy;

    f_r = cameraParams_r.focal_length;
    cx_r = cameraParams_r.cx;
    cy_r = cameraParams_r.cy;

    baseline = cameraParams_l.baseline;
    
    for m = 1:num_points
        if isnan(pixel_coords_left(1, m)) || isnan(pixel_coords_right(1, m))
            points_3D(:, m) = [NaN; NaN; NaN];
            continue;
        end
        
        u_left = pixel_coords_left(1, m);
        v_left = pixel_coords_left(2, m);
        
        u_right = pixel_coords_right(1, m);
        
        Z = f_l * f_r * baseline / (f_l*(cx_r - u_right) - f_r*(cx_l - u_left));
        X = (u_left - cx_l) * Z / f_l;
        Y = (v_left - cy_l) * Z / f_l;
        
        points_3D(:, m) = [X; Y; Z];
    end
end

function [R, t] = calculateTransformation(X,X_prev)
% X = [x1,x2,...,xN; y1,y2,...,yN; z1,z2,...,zN],
% where each column represents coordinates of each feature

    p_prev = mean(X_prev,2);    % computing centroid
    p = mean(X,2);

    Q_prev = X_prev - p_prev;   % Removing centroid
    Q = X - p;

    H = Q_prev * Q';

    [U,S,V] = svd(H);

    R = V * U';

    if det(R) < 0
        if abs(S(2, 2)) < 1e-5 && abs(S(3, 3)) < 1e-5   
            disp('Cannot proceed by this method');
        elseif abs(S(3, 3)) < 1e-5      % Coplanar points case
            V(:, 3) = -V(:, 3);
            R = V * U';
        end
    end
    
    t = p - R*p_prev;   % Calculating translation from optimum rotation
end 

function visualizeCameraPoseSimple(t, R_cam_meas_all, t_cam_meas_all)

    N = length(R_cam_meas_all);

    eul = zeros(3, N);
    trans = zeros(3, N);

    for i = 1:N
        R = R_cam_meas_all{i};
        eul(1, i) = atan2(R(1,2),R(1,1));
        eul(2, i) = -asin(R(1,3));
        eul(3, i) = atan2(R(2,3), R(3,3));
        trans(:, i) = t_cam_meas_all{i};
    end

    figure('Name', 'Camera Orientation vs Time', 'Position', [100 100 900 400]);
    plot(t, rad2deg(eul), 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    legend('Yaw', 'Pitch', 'Roll');
    title('Estimated Camera Orientation');              
    grid on;

    figure('Name', 'Camera Translation vs Time', 'Position', [100 550 900 400]);
    plot(t, trans, 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Translation [m]');
    legend('X', 'Y', 'Z');
    title('Estimated Camera Translation');
    grid on;
end

function R = eul2rotmat(angles, seq)
%EUL2ROTMAT  Convert Euler Angles of given sequence to DCM
    
    N = size(angles,2);
    R = zeros(3,3,N);
    for i = 1:N
        if seq == 'xyx'
            C = singleAxisDCM(1,angles(3,i))*singleAxisDCM(2,angles(2,i))*singleAxisDCM(1,angles(1,i));
        elseif seq == 'xyz'
            C = singleAxisDCM(3,angles(3,i))*singleAxisDCM(2,angles(2,i))*singleAxisDCM(1,angles(1,i));
        elseif seq == 'xzx'
            C = singleAxisDCM(1,angles(3,i))*singleAxisDCM(3,angles(2,i))*singleAxisDCM(1,angles(1,i));
        elseif seq == 'xzy'
            C = singleAxisDCM(2,angles(3,i))*singleAxisDCM(3,angles(2,i))*singleAxisDCM(1,angles(1,i));
        elseif seq == 'yxy'
            C = singleAxisDCM(2,angles(3,i))*singleAxisDCM(1,angles(2,i))*singleAxisDCM(2,angles(1,i));
        elseif seq == 'yxz'
            C = singleAxisDCM(3,angles(3,i))*singleAxisDCM(1,angles(2,i))*singleAxisDCM(2,angles(1,i));
        elseif seq == 'yzx'
            C = singleAxisDCM(1,angles(3,i))*singleAxisDCM(3,angles(2,i))*singleAxisDCM(2,angles(1,i));
        elseif seq == 'yzy'
            C = singleAxisDCM(2,angles(3,i))*singleAxisDCM(3,angles(2,i))*singleAxisDCM(2,angles(1,i));
        elseif seq == 'zxy'
            C = singleAxisDCM(2,angles(3,i))*singleAxisDCM(1,angles(2,i))*singleAxisDCM(3,angles(1,i));
        elseif seq == 'zxz'
            C = singleAxisDCM(3,angles(3,i))*singleAxisDCM(1,angles(2,i))*singleAxisDCM(3,angles(1,i));
        elseif seq == 'zyx'
            C = singleAxisDCM(1,angles(3,i))*singleAxisDCM(2,angles(2,i))*singleAxisDCM(3,angles(1,i));
        elseif seq == 'zyz'
            C = singleAxisDCM(3,angles(3,i))*singleAxisDCM(2,angles(2,i))*singleAxisDCM(3,angles(1,i));
        else 
            error('Invalid sequence')
        end
        R(:,:,i) = C;
    end
end

function C = singleAxisDCM(axis,angle)
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