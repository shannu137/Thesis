function wps_m = get_astar_wp(start, goal, terrain, params)
% Returns way points [x,y] coordinates in terrain frame (m) 

    % Cost weights
    w_d     = params.cost_w_distance;
    w_turn  = params.cost_w_turn;
    w_slope = params.cost_w_slope;
    w_rough = params.cost_w_rough;

    % Limits
    slope_limit = params.rover_slope_limit;
    rough_limit = params.rover_rough_limit;
    gnd_clr     = params.rover_gnd_clr;

    % Grid
    Z_grid = terrain.Z;
    Ngrid  = size(Z_grid,1);

    xs = terrain.X(1,:);
    ys = terrain.Y(:,1)';
    dx = xs(2) - xs(1);  
    dy = ys(2) - ys(1);

    % Slope and Roughness
    slope_grid   = zeros(Ngrid,Ngrid);
    rough_grid   = zeros(Ngrid,Ngrid);
    delta_z_grid = zeros(Ngrid,Ngrid);

    for i=1:Ngrid
        for j=1:Ngrid
            [slope_grid(j,i), rough_grid(j,i), delta_z_grid(j,i)] = compute_slope_roughness(i,j,Z_grid,xs,ys,params);
        end
    end

    % Block cells with exceeding max_slope and gnd_clr limits
    blocked = slope_grid > slope_limit | rough_grid > rough_limit | delta_z_grid >= gnd_clr;

    % Moves
    moves = [-1,-1; -1,0; -1,1;
              0,-1;        0,1;
              1,-1;  1,0;  1,1];
    
    n_dirs = size(moves,1);

    % Unblock start and goal cells
    blocked(start(2), start(1)) = false;
    blocked(goal(2),  goal(1))  = false;

    % A*
    INF = 1e10;
    g_score = INF*ones(Ngrid,Ngrid,n_dirs);
    f_score = INF*ones(Ngrid,Ngrid,n_dirs);
    closed   = false(Ngrid,Ngrid,n_dirs);

    % parent: [pi,pj,pk]
    parent   = zeros(Ngrid,Ngrid,n_dirs,3);

    % Priority Queue: [f,i,j,k]
    openPQ = [];
    best_goal_cost  = INF;
    best_goal_state = [];

    % Initialize start (all directions)
    for k = 1:n_dirs
        g_score(start(2),start(1),k) = 0;
        f_score(start(2),start(1),k) = heuristic(start,goal,dx,dy,params);
        openPQ = [openPQ; f_score(start(2),start(1),k), start(1), start(2), k];
    end

    while ~isempty(openPQ)
        % pop min (PQ)
        [~,idx] = min(openPQ(:,1));
        current = openPQ(idx,:);
        openPQ(idx,:) = [];
    
        ci = current(2); cj = current(3); ck = current(4);

        if closed(cj,ci,ck)
            continue;
        end
        closed(cj,ci,ck) = true;

        if current(1) >= best_goal_cost
            break;
        end

        % Goal check
        if ci == goal(1) && cj == goal(2)
            if g_score(cj,ci,ck) < best_goal_cost
                best_goal_cost = g_score(cj,ci,ck);
                best_goal_state = [ci,cj,ck];
            end
            continue;
        end

        % Explore neighbours
        for k = 1:n_dirs
            ni = ci + moves(k,1);
            nj = cj + moves(k,2);

            if ni<1 || nj<1 || ni>Ngrid || nj>Ngrid
                continue;
            end

            if blocked(nj,ni) || closed(nj,ni,k)
                continue;
            end

            % Distance cost
            d = sqrt((moves(k,1)*dx)^2 + (moves(k,2)*dy)^2);

            % Turning cost
            v1 = moves(ck,:); v2 = moves(k,:);
            cos_psi = dot(v1,v2) / (norm(v1)*norm(v2));
            dpsi = acos(max(-1,min(1,cos_psi)));

            % Terrain
            slope_val = slope_grid(nj,ni);
            rough_val = rough_grid(nj,ni);

            cost = w_d*d + w_turn*dpsi + w_slope*abs(slope_val)*d + w_rough*abs(rough_val)*d;
            tentative_g = g_score(cj,ci,ck) + cost;

            if tentative_g < g_score(nj,ni,k)
                g_score(nj,ni,k) = tentative_g;
                f_score(nj,ni,k) = tentative_g + heuristic([ni,nj],goal,dx,dy,params);

                % fprintf('d: %.4f | turn: %.4f | slope: %.4f | rough: %.4f\n', ...
                %     w_d*d, ...
                %     w_turn*dpsi, ...
                %     w_slope*abs(slope_val)*d, ...
                %     w_rough*abs(rough_val)*d);

                parent(nj,ni,k,:) = [ci,cj,ck];

                % push into PQ
                openPQ = [openPQ; f_score(nj,ni,k),ni,nj,k];
            end
        end
    end

    if isempty(best_goal_state)
        warning('A*: Goal unreachable.');
        wps = [];
        return;
    end

    % Path
    wps = [];
    node = best_goal_state;

    while true
        wps = [node(1:2); wps];

        if node(1)==start(1) && node(2)==start(2)
            break;
        end

        p = squeeze(parent(node(2),node(1),node(3),:))';
        node = p;
    end

    % way points coordinates in terrain frame (m)
    wps_m = [xs(wps(:,1))', ys(wps(:,2))'];

    % Cost map
    costmap = zeros(Ngrid, Ngrid);
    
    for i = 1:Ngrid
        for j = 1:Ngrid
    
            if blocked(j,i)
                costmap(j,i) = NaN; % obstacles
            else
                costmap(j,i) = ...
                    w_slope * abs(slope_grid(j,i)) + ...
                    w_rough * abs(rough_grid(j,i));
            end
    
        end
    end
    
    figure;
    imagesc(xs, ys, costmap);
    set(gca,'YDir','normal');
    colormap(jet);
    colorbar;
    axis equal tight;
    title('A* Costmap');
    xlabel('X'); ylabel('Y');
    hold on;

    % Blocked nodes
    [by, bx] = find(blocked);
    plot(xs(bx), ys(by), 'ks', 'MarkerFaceColor','k');
    
    % path
    plot(wps_m(:,1), wps_m(:,2), ...
        'w', 'LineWidth', 2);
    
    % start and goal
    plot(xs(start(1)), ys(start(2)), 'go', 'MarkerSize',8,'LineWidth',2);
    plot(xs(goal(1)),  ys(goal(2)),  'rx', 'MarkerSize',10,'LineWidth',2);

    % Path
    figure; surf(terrain.X,terrain.Y,terrain.Z); hold on;
    shading interp 
    colormap(gray) 
    axis equal
    title('A* path');
    plot3(wps_m(:,1), wps_m(:,2), ...
          arrayfun(@(k) Z_grid(wps(k,2),wps(k,1)),1:size(wps,1))' + 0.05,...
          'r','LineWidth',2);
end

function h = heuristic(n, g, dx, dy, params)
    d = sqrt(((n(1)-g(1))*dx)^2 + ((n(2)-g(2))*dy)^2);
    h = params.cost_w_distance * d;
end

function [slope, roughness, delta_z] = compute_slope_roughness(i,j,Z_grid,xs,ys,params)
    dx = xs(2) - xs(1);
    dy = ys(2) - ys(1);

    rx = round(params.rover_length/(2*dx));
    ry = round(params.rover_width /(2*dy));

    i1=max(1,i-rx); i2=min(length(xs),i+rx);
    j1=max(1,j-ry); j2=min(length(ys),j+ry);

    X_pts=[]; Y_pts=[]; Z_pts=[];

    for ii = i1:i2
        for jj = j1:j2
            X_pts(end+1) = xs(ii);
            Y_pts(end+1) = ys(jj);
            Z_pts(end+1) = Z_grid(jj,ii);
        end
    end

    % ground clearance
    delta_z = max(Z_pts) - min(Z_pts);

    % Fitted plane
    A = [X_pts', Y_pts', ones(length(X_pts),1)];
    coeffs = A \ Z_pts';
    a0 = coeffs(1);
    a1 = coeffs(2);

    % Slope
    slope = acos(1/sqrt(a0^2 + a1^2 + 1));

    % Roughness
    zfit = A*coeffs;
    roughness = mean(abs(zfit-Z_pts')/sqrt(a0^2+a1^2+1));
end