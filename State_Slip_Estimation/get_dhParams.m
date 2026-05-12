function [traj, dhParams] = get_dhParams(traj, terrain, params)
    t = traj.t;
    
    z        = zeros(size(t));
    roll     = zeros(size(t));
    pitch    = zeros(size(t));
    zdot     = zeros(size(t));
    zddot    = zeros(size(t));
    rolldot  = zeros(size(t));
    pitchdot = zeros(size(t));
    contact_points = zeros(3,6,length(t));
    
    dhParams(length(t)) = struct( ...
        'beta',     0, ...
        'betadot',  0, ...
        'rho1',     0, ...
        'rho1dot',  0, ...
        'rho2',     0, ...
        'rho2dot',  0, ...
        'psi',      zeros(6,1), ...
        'psidot',   zeros(6,1), ...
        'delta',    zeros(6,1), ...
        'deltadot', zeros(6,1), ...
        'thetadot_cmd', zeros(6,1) ...
    );
    
    alpha     = params.ema_alpha; 
    alpha_der = params.ema_alpha_der; 
    
    [z_terr, dzdx, dzdy] = terrain.query(traj.x(1,1), traj.x(2,1));
    
    psi_k = zeros(6,1); delta_k = zeros(6,1);
    beta_k = 0; rho1_k = 0; rho2_k = 0;
    z_k = 0; zdot_k = 0; roll_k = 0; pitch_k = 0;
    prev_sol = [z_terr + params.rover_gnd_clr; ...
                atan(dzdx); atan(dzdy); ...
                atan(dzdy) / 2; atan(dzdy) / 4; atan(dzdy) / 4; ...
                repmat(atan(dzdy), 6, 1)];
    
    for i = 1:length(t)
        disp(i)
        [sol, contact_point] = solve_rover_pose([traj.x(:,i); traj.yaw_des(i); psi_k], prev_sol, terrain, params);
    
        contact_points(:,:,i) = contact_point;
        
        if i > 1
            z(i)     = alpha*sol(1) + (1-alpha)*z_k;
            roll(i)  = alpha*sol(2) + (1-alpha)*roll_k;
            pitch(i) = alpha*sol(3) + (1-alpha)*pitch_k;
    
            dhParams(i).beta = alpha*sol(4) + (1-alpha)*beta_k;
            dhParams(i).rho1 = alpha*sol(5) + (1-alpha)*rho1_k;
            dhParams(i).rho2 = alpha*sol(6) + (1-alpha)*rho2_k;
    
            dhParams(i).delta = alpha*sol(7:12) + (1-alpha)*delta_k;
    
            dt = t(i) - t(i-1);
            raw_zdot     = (z(i) - z_k)/dt;
            raw_rolldot  = (roll(i) - roll_k)/dt;
            raw_pitchdot = (pitch(i) - pitch_k)/dt;
            
            raw_betadot = (dhParams(i).beta - beta_k)/dt;
            raw_rho1dot = (dhParams(i).rho1 - rho1_k)/dt;
            raw_rho2dot = (dhParams(i).rho2 - rho2_k)/dt;
            
            raw_deltadot = (dhParams(i).delta - delta_k)/dt;
    
            zdot(i)     = alpha_der*raw_zdot  + (1-alpha_der)*zdot(i-1);
            rolldot(i)  = alpha_der*raw_rolldot  + (1-alpha_der)*rolldot(i-1);
            pitchdot(i) = alpha_der*raw_pitchdot + (1-alpha_der)*pitchdot(i-1);
        
            dhParams(i).betadot = alpha_der*raw_betadot + (1-alpha_der)*dhParams(i-1).betadot;
            dhParams(i).rho1dot = alpha_der*raw_rho1dot + (1-alpha_der)*dhParams(i-1).rho1dot;
            dhParams(i).rho2dot = alpha_der*raw_rho2dot + (1-alpha_der)*dhParams(i-1).rho2dot;
        
            dhParams(i).deltadot = alpha_der*raw_deltadot + ...
                                   (1-alpha_der)*dhParams(i-1).deltadot;

            raw_zddot = (zdot(i) - zdot_k)/dt;
            zddot(i)  = alpha_der*raw_zddot  + (1-alpha_der)*zddot(i-1);
        else
            z(i)     = sol(1);
            roll(i)  = sol(2);
            pitch(i) = sol(3);
        
            dhParams(i).beta = sol(4);
            dhParams(i).rho1 = sol(5);
            dhParams(i).rho2 = sol(6);
        
            dhParams(i).delta = sol(7:12);

            dhParams(i).betadot = 0;
            dhParams(i).rho1dot = 0;
            dhParams(i).rho2dot = 0;
        
            dhParams(i).deltadot = 0;
        end
    
        if i > 1
            dhParams(i).psi      = dhParams(i-1).psi;
            dhParams(i).thetadot_cmd = dhParams(i-1).thetadot_cmd;
            dhParams(i).psidot   = dhParams(i-1).psidot;
        else
            dhParams(i).psi      = zeros(6,1);
            dhParams(i).thetadot_cmd = zeros(6,1);
            dhParams(i).psidot   = zeros(6,1);
        end

        [psi,thetadot] = inverse_kinematics(traj.des_states(:,i), [rolldot(i), pitchdot(i)], params, dhParams(i));
        dhParams(i).thetadot_cmd = thetadot;
    
        if i > 1
            dhParams(i).psi = alpha*psi + (1-alpha)*psi_k;
            dpsi = wrapToPi(dhParams(i).psi - psi_k);
            raw_psidot = dpsi / dt;
            dhParams(i).psidot = alpha_der*raw_psidot + ...
                                 (1-alpha_der)*dhParams(i-1).psidot;
        else 
            dhParams(i).psi = psi;
        end
    
        psi_k = dhParams(i).psi; delta_k = dhParams(i).delta;
        beta_k = dhParams(i).beta; rho1_k = dhParams(i).rho1; rho2_k = dhParams(i).rho2;
        z_k = z(i); zdot_k = zdot(i); roll_k = roll(i); pitch_k = pitch(i);
        prev_sol = sol;
    end

    traj.x         = [traj.x; z];
    traj.v         = [traj.v; zdot];
    traj.a         = [traj.a; zddot];
    traj.euler     = [roll; pitch; traj.yaw_des];
    traj.eulerdots = [rolldot; pitchdot; traj.des_states(3,:)];
end