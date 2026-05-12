function [accelMeas, gyroMeas] = simulate_imu(actual_traj, params)
    N = length(actual_traj.t);
    omega = zeros(3,N);
    R_N2B = zeros(3,3,N);
    
    for k = 1:N
        phi   = actual_traj.euler(1,k);
        theta = actual_traj.euler(2,k);
    
        T = [ ...
            1 0 -sin(theta);
            0 cos(phi) sin(phi)*cos(theta);
            0 -sin(phi) cos(phi)*cos(theta)];
    
        R_N2B(:,:,k) = singleAxisDCM(1,phi) * singleAxisDCM(2,theta) * singleAxisDCM(3,actual_traj.euler(3,k));
    
        omega_body = T * actual_traj.eulerdots(:,k);
    
        omega(:,k) = R_N2B(:,:,k)' * omega_body;
    end
    
    acc = actual_traj.a - [0; 0; 9.81-params.gravity];
    
    [accelMeas, gyroMeas] = params.imu(acc', omega', R_N2B);
    
    accelMeas = accelMeas';
    gyroMeas = gyroMeas';
end