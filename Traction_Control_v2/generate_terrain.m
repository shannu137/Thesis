function terrain = generate_terrain(params, showTerrain)

    gnd_clr = params.rover_gnd_clr;          % meters
    N       = params.terrain_res_oct;        % controls terrain resolution
    
    Lx = params.terrain_Lx;                  % meters
    Ly = params.terrain_Ly;                  % meters
    
    numCraters = params.terrain_numCraters;  % number of craters
    numHills   = params.terrain_numHills;    % number of hills
    
    size_grid  = 2^N + 1;                    % grid size
    [xg,yg]    = meshgrid(linspace(0,Lx,size_grid), ...
                       linspace(0,Ly,size_grid));
    
    Z = zeros(size_grid);
    
    %% Diamond-Square Terrain Generation
    % Initialize corners
    Z(1,1)     = rand;
    Z(1,end)   = rand;
    Z(end,1)   = rand;
    Z(end,end) = rand;
    
    step  = size_grid - 1;
    scale = 1;
    
    while step > 1
        half = step / 2;
    
        % Diamond Step
        for x = 1:step:size_grid-1
            for y = 1:step:size_grid-1
                avg = ( Z(x,y) + Z(x+step,y) + ...
                        Z(x,y+step) + Z(x+step,y+step) ) / 4;
    
                Z(x+half,y+half) = avg + scale*(rand-0.5);
            end
        end
    
        % Square Step
        for x = 1:half:size_grid
            for y = mod(x+half-1,step)+1:step:size_grid
                s = [];
    
                if x-half > 0
                    s(end+1) = Z(x-half,y);
                end
    
                if x+half <= size_grid
                    s(end+1) = Z(x+half,y);
                end
    
                if y-half > 0
                    s(end+1) = Z(x,y-half);
                end
    
                if y+half <= size_grid
                    s(end+1) = Z(x,y+half);
                end
    
                avg = mean(s);
    
                Z(x,y) = avg + scale*(rand-0.5);
            end
        end
    
        step = half;
        scale = scale * 0.5;
    end
    
    Z = Z - mean(Z(:));         % mean = 0
    Z = Z / max(abs(Z(:)));     % normalizing to [-1,1]
    Z = gnd_clr * 0.8 * Z;      % scaling so that rover can traverse
    
    %% Adding Craters and Hills
    for k = 1:numCraters
        % crater center
        cxC = rand * Lx;
        cyC = rand * Ly;
    
        % crater radius
        if randn < 0.9
            RC = 0.1 + 0.15*rand;
        else
            RC = 0.15 + 0.35*rand;
        end
    
        % crater depth
        depthC = 0.5*RC;
    
        % distance field
        rC = sqrt((xg-cxC).^2 + (yg-cyC).^2);
    
        % crater bowl
        crater = -depthC * exp(-(rC.^2)/(RC^2));
    
        % crater rim
        rim = 0.25*depthC * exp(-(rC.^2)/(1.6*RC^2));
    
        % apply crater
        Z = Z + crater + rim;
    end
    
    for k = 1:numHills
        % hill center
        cxH = rand * Lx;
        cyH = rand * Ly;
    
        % hill radius
        if randn < 0.9
            RH = 0.1 + 0.15*rand;
        else
            RH = 0.15 + 0.35*rand;
        end
    
        % hill height
        depthH = 0.5*RH;
    
        % distance field
        rH = sqrt((xg-cxH).^2 + (yg-cyH).^2);
    
        % hill
        hill   =  depthH * exp(-(rH.^2)/(RH^2));
    
        % apply hill
        Z = Z + hill;
    end
    
    %% Terrain smoothing
    for k = 1:3
        Z = ( ...
            Z + ...
            circshift(Z,[1 0]) + circshift(Z,[-1 0]) + ...
            circshift(Z,[0 1]) + circshift(Z,[0 -1]) ) / 5;
    end
    
    %% Terrain Gradient
    resx = Lx/(size_grid-1);
    resy = Ly/(size_grid-1);
    
    [dZdx,dZdy] = gradient(Z,resx,resy);
    
    %% Interpolants
    z  = griddedInterpolant(xg',yg',Z,'spline');
    Zx = griddedInterpolant(xg',yg',dZdx,'spline');
    Zy = griddedInterpolant(xg',yg',dZdy,'spline');
    
    %% Output
    terrain.X = xg;
    terrain.Y = yg;
    terrain.Z = Z;
    
    % query(x,y) -> [z, dzdx, dzdy]
    terrain.query = @(x,y) query_terrain(x, y, z, Zx, Zy);
    
    %% Plot Terrain
    if (showTerrain)
        surf(xg,yg,Z) 
        shading interp 
        colormap(gray) 
        axis equal
        xlabel('x (m)'); ylabel('y (m)'); zlabel('height (m)')
    end
end
%% 
function [z, dzdx, dzdy] = query_terrain(x,y,Z,Zx,Zy)
    z    =  Z(x, y);
    dzdx = Zx(x, y);
    dzdy = Zy(x, y);
end