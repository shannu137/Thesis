function traj = generateTrajectory(parameters)
            
    trajectoryType = parameters.trajectoryType;
    
    if strcmp(trajectoryType, 'mixed')
        type = rand() < (1/8);
        trajType = {'amateur', 'professional'};
        trajType = trajType{type + 1};
    else
        trajType = trajectoryType;
    end
    
    if strcmp(trajType, 'amateur')
        traj  = generateAmateurTrajectory(parameters);
    else
        traj = generateProfessionalTrajectory(parameters);
    end

    traj = getAngVel(traj);
    traj = addProcessNoise(traj, parameters);
    traj = createGlobalOrientation(traj);

    traj.type = trajType;
    traj.duration = parameters.duration;
end

function traj = generateProfessionalTrajectory(parameters)
    
    dt = 1 / parameters.samplingRate;
    workspaceSize = parameters.workspaceSize;
    numWaypoints = randi([3, 7]);
    
    waypoints = zeros(3, numWaypoints + 1); 
    
    for i = 2:numWaypoints + 1
        validWaypoint = false;
        
        while ~validWaypoint
            waypoint = [rand() * workspaceSize(1);
                        rand() * workspaceSize(2);
                        rand() * workspaceSize(3)];
            
            distance = norm(waypoint - waypoints(:, i-1));
            minDistance = max(workspaceSize) / 4;
            
            if distance >= minDistance
                validWaypoint = true;
                waypoints(:, i) = waypoint;
            end
        end
    end

    yawAngles = zeros(1, numWaypoints);
    maneuverAxes = zeros(1, numWaypoints);
    theta_values = zeros(2, numWaypoints);

    totalYawRotation = 0;
    totalManeuverRotation = 0;
    currentYaw = 0;
    
    for i = 1:numWaypoints
        direction = waypoints(:, i+1) - waypoints(:, i);
        yawAngles(i) = atan2(direction(2), direction(1));
        
        if rand() < 0.5
            maneuverAxes(i) = 1;
        else
            maneuverAxes(i) = 2;
        end
        
        theta_degrees = 5 + rand(1, 2) * 40;
        theta_degrees = round(theta_degrees, 1);
        theta_values(:, i) = theta_degrees' * pi / 180;

        totalYawRotation = totalYawRotation + abs(currentYaw - yawAngles(i));
        currentYaw = yawAngles(i);
        totalManeuverRotation = totalManeuverRotation + 2 * sum(theta_values(:, i));
    end
    
    distances = vecnorm(diff(waypoints, 1, 2), 2, 1);
    totalDistance = sum(distances);
    totalRotation = totalYawRotation + totalManeuverRotation;
    
    totalSamples      = round(parameters.duration / dt);
    rotationSamples   = round(0.3 * totalSamples);
    translationSamples = totalSamples - rotationSamples;
    samplesRemaining = totalSamples;
    
    currentPos = waypoints(:, 1);
    currentRot = [0; 0; 0]; 
    
    acc = [0; 0; 0];
    eul = currentRot;
    angvel = [0; 0; 0];
    time = 0;
    timeOffset = 0;
    
    for i = 1:numWaypoints        
        targetRot = currentRot;
        targetRot(3) = yawAngles(i);

        yawSamples = round((abs(targetRot(3) - currentRot(3)) / totalRotation) * rotationSamples);
        yawSamples = min(yawSamples, samplesRemaining);

        yawDuration = yawSamples * dt;

        if yawDuration > 1e-6
            samplesRemaining = samplesRemaining - yawSamples;

            [t, p, v, ~] = quinticTrajectory(currentRot, targetRot, yawDuration, parameters.samplingRate);
            
            time = [time, t(2:end) + timeOffset];
            % pos = [pos, repmat(currentPos, 1, length(t)-1)];
            % vel = [vel, zeros(3, length(t)-1)];
            acc = [acc, zeros(3, length(t)-1)];
            eul = [eul, p(:, 2:end)];
            angvel = [angvel, v(:, 2:end)];
            
            timeOffset = timeOffset + yawDuration;
            currentRot = targetRot;
        end
        
        segmentSamples = round((distances(i) / totalDistance) * translationSamples);
        segmentSamples = min(segmentSamples, samplesRemaining);
        samplesRemaining = samplesRemaining - segmentSamples;

        segmentDuration = segmentSamples * dt;
        nextPos = waypoints(:, i+1);
        
        [t, ~, ~, a] = quinticTrajectory(currentPos, nextPos, segmentDuration, parameters.samplingRate);
        
        time = [time, t(2:end) + timeOffset];
        % pos = [pos, p(:, 2:end)];
        % vel = [vel, v(:, 2:end)];
        acc = [acc, a(:, 2:end)];
        eul = [eul, repmat(currentRot, 1, length(t)-1)];
        angvel = [angvel, zeros(3, length(t)-1)];
        
        timeOffset = timeOffset + segmentDuration;
        currentPos = nextPos;
        
        maneuverAxis = maneuverAxes(i);
        theta = theta_values(:, i);
        
        % Maneuver: 0 -> theta1 -> 0 -> -theta2 -> 0
        maneuverWaypoints = [0; theta(1); 0; -theta(2); 0];
        totalManeuverAngle = 2 * sum(theta);
        maneuverSamples = round((totalManeuverAngle / totalRotation) * rotationSamples);

        for j = 1:4
            targetRot = currentRot;
            targetRot(maneuverAxis) = maneuverWaypoints(j+1);

            if i == numWaypoints && j == 4
                subSegmentSamples = samplesRemaining;
            else
                subSegmentSamples = round((abs(maneuverWaypoints(j+1) - maneuverWaypoints(j)) / totalManeuverAngle) * maneuverSamples);
                subSegmentSamples = min(subSegmentSamples, samplesRemaining);
                samplesRemaining  = samplesRemaining - subSegmentSamples;
            end
            subSegmentDuration = subSegmentSamples * dt;
            
            [t, p, v, ~] = quinticTrajectory(currentRot, targetRot, subSegmentDuration, parameters.samplingRate);
            
            time = [time, t(2:end) + timeOffset];
            % pos = [pos, repmat(currentPos, 1, length(t)-1)];
            % vel = [vel, zeros(3, length(t)-1)];
            acc = [acc, zeros(3, length(t)-1)];
            eul = [eul, p(:, 2:end)];
            angvel = [angvel, v(:, 2:end)];
            
            timeOffset = timeOffset + subSegmentDuration;
            currentRot = targetRot;
        end
    end
    traj = struct('t', time, 'acc', acc, 'eul', eul, 'angvel', angvel);
end

function traj = generateAmateurTrajectory(parameters)
    if rand < (1/7)
        traj = generate_sweep_traj(parameters);
    else
        traj = generate_single_axis_traj(parameters);
    end
end

function traj = generate_sweep_traj(parameters)

    dt = 1 / parameters.samplingRate;
    workspaceSize = parameters.workspaceSize;

    numSweeps = randi([3, 5]);
    totalSamples = round(parameters.duration / dt);
    samplesRemaining = totalSamples;
    % segmentDuration = round(parameters.duration / numSweeps, 1); 
    
    if rand() < 0.5
        sweepAxis = 1;
        slideAxis = 2;
    else
        sweepAxis = 2;
        slideAxis = 1;
    end

    currentPos = [0; 0; 0];
    acc = [0; 0; 0];
    time = 0;
    timeOffset = 0;

    for i = 1:numSweeps
        sweepEnd = currentPos;
        if mod(i, 2) == 1
            sweepEnd(sweepAxis) = workspaceSize(sweepAxis) * (0.85 + rand() * 0.15);
        else
            sweepEnd(sweepAxis) = workspaceSize(sweepAxis) * rand() * 0.15;
        end

        if i == numSweeps
            sweepSamples = samplesRemaining;
        else
            sweepSamples = round(0.85 * totalSamples / numSweeps);
            sweepSamples = min(sweepSamples, samplesRemaining);
            samplesRemaining = samplesRemaining - sweepSamples;
        end
        sweepDuration = sweepSamples * dt;
        
        [t, ~, ~, a] = quinticTrajectory(currentPos, sweepEnd, sweepDuration, parameters.samplingRate);

        time = [time, t(2:end) + timeOffset];
        % pos = [pos, p(:,2:end)];
        % vel = [vel, v(:,2:end)];
        acc = [acc, a(:,2:end)];

        timeOffset = timeOffset + sweepDuration;
        currentPos = sweepEnd;
        
        if i < numSweeps
            slideEnd = currentPos;
            slideEnd(slideAxis) = slideEnd(slideAxis) + workspaceSize(slideAxis) / (numSweeps);
                    
            slideSamples = round(0.15 * totalSamples / numSweeps);
            slideSamples = min(slideSamples, samplesRemaining);
            samplesRemaining = samplesRemaining - slideSamples;

            slideDuration = slideSamples * dt;

            [t, ~, ~, a] = quinticTrajectory(currentPos, slideEnd, slideDuration, parameters.samplingRate);

            time = [time, t(2:end) + timeOffset];
            % pos = [pos, p(:,2:end)];
            % vel = [vel, v(:,2:end)];
            acc = [acc, a(:,2:end)];

            timeOffset = timeOffset + slideDuration;
            currentPos = slideEnd;
        end
    end 
    traj = struct('t', time, 'acc', acc, 'eul', zeros(3,length(time)), 'angvel', zeros(3,length(time)));
end

function traj = generate_single_axis_traj(parameters)

    dt = 1 / parameters.samplingRate;
    axis = randi(3);

    totalSamples = round(parameters.duration / dt);
    samplesRemaining = totalSamples;
    
    if rand() < 0.5
        minLength = parameters.workspaceSize(axis) / 4;
        maxPos = parameters.workspaceSize(axis);
        
        valid = false;
        while ~valid
            point1 = rand() * maxPos;
            point2 = rand() * maxPos;
            if abs(point2 - point1) >= minLength
                valid = true;
            end
        end
        
        % Trajectory: start -> point1 -> point2 -> start
        numSegments = 3;        
        waypoints = [0; point1; point2; 0];
        totalLength = abs(point1) + abs(point2 - point1) + abs(point2);
        
        currentPos = [0; 0; 0];
        acc = [0; 0; 0];
        time = 0;
        timeOffset = 0;
        
        for i = 1:numSegments
            nextPos = currentPos;
            nextPos(axis) = waypoints(i + 1);

            if i == numSegments
                segmentSamples = samplesRemaining;
            else
                segmentSamples = round((abs(waypoints(i+1) - waypoints(i)) / totalLength) * totalSamples);
                segmentSamples = min(segmentSamples, samplesRemaining);
                samplesRemaining = samplesRemaining - segmentSamples;
            end
            segmentDuration = segmentSamples * dt;
            
            [t, ~, ~, a] = quinticTrajectory(currentPos, nextPos, segmentDuration, parameters.samplingRate);
            
            time = [time, t(2:end) + timeOffset];
            % pos = [pos, p(:, 2:end)];
            % vel = [vel, v(:, 2:end)];
            acc = [acc, a(:, 2:end)];
            
            timeOffset = timeOffset + segmentDuration;
            currentPos = nextPos;
        end
        
        eul = zeros(3, length(time));
        angvel = zeros(3, length(time));

    else
        theta_degrees = 5 + rand(1,2) * 40;
        theta_degrees = round(theta_degrees, 1);
        theta = theta_degrees * pi / 180;
        
        % Trajectory: 0 -> theta1 -> 0 -> -theta2 -> 0
        numSegments = 4;        
        waypoints = [0; theta(1); 0; -theta(2); 0];
        
        currentAngles = [0; 0; 0];
        eul = currentAngles;
        angvel = [0; 0; 0];
        time = 0;
        timeOffset = 0;
        
        for i = 1:numSegments
            nextAngles = currentAngles;
            nextAngles(axis) = waypoints(i + 1);
            
            if i == numSegments
                segmentSamples = samplesRemaining;
            else
                segmentSamples = round((abs(waypoints(i+1) - waypoints(i)) / (2*sum(theta))) * totalSamples);
                segmentSamples = min(segmentSamples, samplesRemaining);
                samplesRemaining = samplesRemaining - segmentSamples;
            end
            segmentDuration = segmentSamples * dt;

            [t, p, v, ~] = quinticTrajectory(currentAngles, nextAngles, segmentDuration, parameters.samplingRate);
            
            time = [time, t(2:end) + timeOffset];
            eul = [eul, p(:, 2:end)];
            angvel = [angvel, v(:, 2:end)];
            
            timeOffset = timeOffset + segmentDuration;
            currentAngles = nextAngles;
        end
        
        % pos = zeros(3, length(time));
        % vel = zeros(3, length(time));
        acc = zeros(3, length(time));

    end
    traj = struct('t', time, 'acc', acc, 'eul', eul, 'angvel', angvel);
end

function [t,p,v,a] = quinticTrajectory(startVal, endVal, duration, samplingRate)

    T = duration;
    t = 0 : (1/samplingRate) : T;
    tau = t/T;
    
    p = startVal + (endVal-startVal) * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    v = (endVal-startVal)/T * (30*tau.^2 - 60*tau.^3 + 30*tau.^4);
    a = (endVal-startVal)/T^2 * (60*tau - 180*tau.^2 + 120*tau.^3);
end

function traj = getAngVel(traj)
    eul = traj.eul;
    angvel = traj.angvel;

    omega = zeros(size(angvel));

    for i = 1:length(traj.t)
        roll = eul(1,i); pitch = eul(2,i); yaw = eul(3,i);
        rollDot = angvel(1,i); pitchDot = angvel(2,i); yawDot = angvel(3,i);

        T = [-sin(pitch) 0 1;
             sin(roll)*cos(pitch) cos(roll) 0;
             cos(roll)*cos(pitch) -sin(roll) 0];

        omega(:,i) = T * [yawDot; pitchDot; rollDot];
    end
    
    traj.eul([1 3],:) = traj.eul([3 1],:);
    traj.angvel = omega;
end

function traj = createGlobalOrientation(traj)
    eul = traj.eul;
    angvel = traj.angvel;

    omega_global = zeros(size(angvel));

    for i = 1:length(traj.t)
        yaw = eul(1,i); pitch = eul(2,i); roll = eul(3,i);

        Rz = [cos(yaw) sin(yaw) 0;
              -sin(yaw)  cos(yaw) 0;
                   0         0   1];
        Ry = [cos(pitch)  0  -sin(pitch);
                   0      1      0;
             sin(pitch) 0  cos(pitch)];
        Rx = [1      0           0;
              0  cos(roll) sin(roll);
              0  -sin(roll)  cos(roll)];

        R_G2B = Rx * Ry * Rz;
        omega_global(:,i) = R_G2B' * angvel(:,i);
    end

    traj.angvel_global = omega_global;
end

function traj = addProcessNoise(traj, parameters)
    Ts = 1 / parameters.samplingRate;

    noise_a = parameters.noiseStd_a * randn(size(traj.acc));
    noise_omega = parameters.noiseStd_omega * randn(size(traj.angvel));

    acc = traj.acc + noise_a;
    omega = traj.angvel + noise_omega;

    pos = zeros(size(traj.acc));
    vel = zeros(size(traj.acc));
    quat = zeros(4, size(traj.angvel,2));

    pos(:,1) = [0; 0; 0];
    vel(:,1) = [0; 0; 0];
    quat(:,1) = eul2quaternion(traj.eul(:,1));

    for i = 2:length(traj.acc)
        vel(:,i) = vel(:,i-1) + acc(:,i-1)*Ts;
        pos(:,i) = pos(:,i-1) + vel(:,i-1)*Ts;
        quat(:,i) = quat(:,i-1) + quat_prop(quat(:,i-1), omega(:,i-1))*Ts;
        quat(:,i) = quat(:,i) / norm(quat(:,i));
    end

    traj.pos = pos;
    traj.vel = vel;
    traj.acc = acc;
    traj.quat = quat;
    traj.eul = quaternion2eul(quat);
    traj.angvel = omega;
end

function quat = quat_prop(quat, omega)
    q0 = quat(1); q1 = quat(2);
    q2 = quat(3); q3 = quat(4);

    R = 0.5 * [-q1 -q2 -q3; 
                q0 -q3  q2;
                q3  q0 -q1;
               -q2  q1  q0];

    quat = R * omega;
end

function quat = eul2quaternion(eul)
    % [yaw; pitch; roll] in radians to quaternions [q0; q1; q2; q2]
    
    roll = eul(3, :);
    pitch = eul(2, :);
    yaw = eul(1, :);
    
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    
    q0 = cr .* cp .* cy + sr .* sp .* sy;
    q1 = sr .* cp .* cy - cr .* sp .* sy;
    q2 = cr .* sp .* cy + sr .* cp .* sy;
    q3 = cr .* cp .* sy - sr .* sp .* cy;
    
    quat = [q0; q1; q2; q3];
    quat = quat ./ vecnorm(quat, 2, 1);
end

function eul = quaternion2eul(quat)
    q0 = quat(1,:); q1 = quat(2,:);
    q2 = quat(3,:); q3 = quat(4,:);

    roll = atan2(2*(q0.*q1 + q2.*q3), 1 - 2*(q1.^2 + q2.^2));
    yaw  = atan2(2*(q0.*q3 + q1.*q2), 1 - 2*(q2.^2 + q3.^2));
    pitch = -pi/2 + 2*atan2(sqrt(1 + 2*(q0.*q2-q1.*q3)) , sqrt(1 - 2*(q0.*q2-q1.*q3)));

    eul = [yaw; pitch; roll];
end