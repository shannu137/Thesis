clear all; close all; clc

params = get_params();
% terrain = generate_terrain(params, true);
terrain = load("terrain_sample2.mat").terrain;

start = [60,90];
goal  = [90,60];
wps   = get_astar_wp(start, goal, terrain, params);
%%
[r,dr,ddr,s]   = build_path(wps, params);
%%
[t,s,sdot,sw,MVC] = topp(dr,ddr,s,params);

fprintf('Total time : %.4f s\n', t(end));
figure('Color','w');

subplot(1,2,1); hold on;
plot(s, sdot, 'b-', 'LineWidth',2);
xlabel('s'); ylabel('v = ds/dt');
title('Velocity profile'); grid on;
plot(s, MVC, 'r', LineWidth=1.5);
legend('v', 'MVC')

subplot(1,2,2);
plot(t, s, 'r-', 'LineWidth',2);
xlabel('t (s)'); ylabel('s');
title('Time law s(t)'); grid on;

%%
sdot = medfilt1(sdot, 5);          % fix before anything else uses it                                         % smooth sdot first
sddot = (sdot(2:end).^2 - sdot(1:end-1).^2) ./ (2 * (diff(s) + 1e-12));
sddot = medfilt1(sddot, 11);                                        % smooth sddot too
sddot(end+1) = sddot(end);

x = r;
v = dr .* sdot;
a = ddr .* (sdot.^2) + dr .* sddot;

%%
figure;
plot(x(:,1), x(:,2), 'b', 'LineWidth', 2);
axis equal;
grid on;
title('Trajectory x(t)');
xlabel('x'); ylabel('y');

figure;
plot(t, x(:,1), 'r', t, x(:,2), 'b');
grid on;
legend('x','y');
title('Position components');
xlabel('t'); ylabel('position');

figure;
plot(t, v(:,1), 'r', t, v(:,2), 'b');
grid on;
legend('v_x','v_y');
title('Velocity components');
xlabel('t'); ylabel('velocity');

a_max = params.rover_acc_limit * ones(size(t));
figure;
plot(t, a(:,1), 'r', t, a(:,2), 'b');hold on
grid on;
legend('a_x','a_y');
title('Acceleration components');
xlabel('t'); ylabel('acceleration');

figure;
plot(t, vecnorm(v,2,2),'k','LineWidth',2);
grid on;
title('Speed ||v(t)||');
xlabel('t'); ylabel('speed');

figure;
plot(t, vecnorm(a,2,2),'b','LineWidth',2);
grid on;
title('Acceleration ||a(t)||');
xlabel('t'); ylabel('acceleration'); ylim([0,0.1])