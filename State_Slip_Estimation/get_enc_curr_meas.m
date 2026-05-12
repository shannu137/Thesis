function [Omega_t, current_t, encoder_m, current_m, Vin] = get_enc_curr_meas(traj, bekker, params, dhParams)
    
    t = traj.t;
    Omega_t   = zeros(6,length(t));
    current_t = zeros(6,length(t));
    encoder_m = zeros(6,length(t));
    current_m = zeros(6,length(t));
    Vin       = zeros(6,length(t));

    Omega_m_k = zeros(6,1);
    theta_m_k = zeros(6,1);
    i_m_k = zeros(6,1);
    e_integral = zeros(6,1);
    
    PWM_max = 255;
    t = traj.t;
    
    Ke = params.motor_Ke;
    b  = params.motor_b;
    Kt = params.motor_Kt;
    J  = params.motor_J;
    L  = params.motor_L;
    R  = params.motor_R;
    r  = params.wheel_radius;
    
    Kff = params.motorctrl_Kff;
    Kp  = params.motorctrl_Kp;
    Ki  = params.motorctrl_Ki;
    
    for i = 1:length(t)
        if i > 1
            dt = t(i) - t(i-1);
        else
            dt = t(2) - t(1);
        end
    
        Omega_cmd = dhParams(i).thetadot_cmd;
        Fsoil = bekker(i).Fsoil;
    
        N_motor = 50;
        dt_motor = dt / N_motor;
    
        for k = 1:N_motor
        
            e = Omega_cmd - Omega_m_k;
            
            u = Kff*Omega_cmd + Kp*e + Ki*e_integral;
            
            integrate_mask = (abs(u) < PWM_max) | (u.*e < 0);
            e_integral(integrate_mask) = e_integral(integrate_mask) + e(integrate_mask)*dt_motor;
            
            u = max(min(u, PWM_max), -PWM_max);
            Vin(:,i) = (u / PWM_max) * params.motor_V_supply;
    
            T_motor = Kt*i_m_k - b*Omega_m_k;
            T_load = r*Fsoil;
        
            idot = (Vin(:,i) - R*i_m_k - Ke*Omega_m_k)/L;
            Omegadot = (T_motor - T_load)/J;
        
            Omega_m_k = Omega_m_k + Omegadot*dt_motor;
            i_m_k = i_m_k + idot*dt_motor;
    
            theta_m_k = theta_m_k + Omega_m_k*dt_motor;
        end
    
        Omega_t(:,i) = Omega_m_k;
        current_t(:,i) = i_m_k * 1000;
        current_m(:,i) = i_m_k * 1000 + params.noise_curr*randn();
        encoder_m(:,i) = round(theta_m_k * params.encoder_CPR / (2*pi) + params.noise_enc*randn());
    end
end