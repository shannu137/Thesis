function plot_traj(traj)
    t = traj.t';
    x = traj.x';
    v = traj.v';
    a = traj.a';
    
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
end