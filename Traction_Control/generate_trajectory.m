fs = params.fs;
dt = 1 / fs;

function traj = plan_trajectory(p_start, p_end, T, terrain, num_samples)
% PLAN_TRAJECTORY  Quintic rest-to-rest trajectory between two 2-D points.
%
%   p_start, p_end  : [x, y] row vectors
%   T               : total travel time (seconds)
%   terrain         : terrain struct from generate_terrain()
%   num_samples     : number of time samples (e.g. 500)
%
%   Boundary conditions:
%       pos(0)  = p_start,  pos(T)  = p_end
%       vel(0)  = [0,0],    vel(T)  = [0,0]
%       acc(0)  = [0,0],    acc(T)  = [0,0]
%
%   Quintic polynomial (per axis):
%       p(t) = p0 + Dp*(10τ³ - 15τ⁴ + 6τ⁵)       τ = t/T
%       v(t) = (Dp/T)*(30τ² - 60τ³ + 30τ⁴)
%       a(t) = (Dp/T²)*(60τ - 180τ² + 120τ³)

    t   = linspace(0, T, num_samples)';   % [N×1] time vector
    tau = t / T;                           % [N×1] normalised time ∈ [0,1]
    Dp  = p_end - p_start;                 % [1×2] displacement vector

    %% Basis polynomials (scalars, applied to each axis via broadcasting)
    b   =  10*tau.^3 - 15*tau.^4 +  6*tau.^5;   % position basis
    db  =  30*tau.^2 - 60*tau.^3 + 30*tau.^4;   % velocity basis  (×1/T)
    ddb =  60*tau    - 180*tau.^2 + 120*tau.^3; % accel basis     (×1/T²)

    %% 2-D kinematics  [N×2]
    pos = p_start + b   .* Dp;
    vel =          (db  .* Dp) / T;
    acc =          (ddb .* Dp) / T^2;

    %% Terrain height & gradient along the path  [N×1]
    [z, dzdx, dzdy] = terrain.query(pos(:,1), pos(:,2));

    %% Pack output
    traj.t    = t;
    traj.pos  = pos;            % [N×2]  x, y
    traj.vel  = vel;            % [N×2]  vx, vy
    traj.acc  = acc;            % [N×2]  ax, ay
    traj.z    = z;              % [N×1]  terrain height at (x,y)
    traj.dzdx = dzdx;           % [N×1]  terrain slope x
    traj.dzdy = dzdy;           % [N×1]  terrain slope y

    %% Derived scalars
    traj.speed = sqrt(sum(vel.^2, 2));     % [N×1]  |v|
    traj.slope = sqrt(dzdx.^2 + dzdy.^2); % [N×1]  terrain steepness

    traj.p_start = p_start;
    traj.p_end   = p_end;
    traj.T       = T;
end