function MPC_plot_results_fun(n_save, print_figures)

data_to_load = sprintf('mpc_data_%d',n_save);
load(data_to_load);

x = x_mpc;
u = u_mpc;

T = data.T;
track = data.track;
d_lim = data.d_lim;
L_track = data.L_track;

K_fun = data.K_fun;
K_fun_real = data.K_fun_real;

%% 1) Trajectory on track

f1 = figure(1); hold on
f1.Position = [200.2000 349.8000 560 420.0000];

[x_c,y_c] = plot_track(track,0,30);
[x_in,y_in] = plot_track(track,d_lim,30);
[x_out,y_out] = plot_track(track,-d_lim,30);

% Track
plot(x_c(1),y_c(1),'k.','markersize',15), axis equal
plot(x_c,y_c,'k-','linewidth',0.5)
plot(x_in,y_in,'k-','linewidth',1)
plot(x_out,y_out,'k-','linewidth',1)
x_axis_u = max(x_out);
x_axis_l = min(x_out);
y_axis_u = max(y_out);
y_axis_l = min(y_out);

title('\textbf{Track}','interpreter','latex')
xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')

% Trajectory
[x_traj, y_traj] = plot_on_track(track,x(11,:),x(12,:));
plot(x_traj,y_traj,'b-','linewidth',1), hold off, grid on

xlim([x_axis_l-0.1, x_axis_u+0.1]);
ylim([y_axis_l-0.1, y_axis_u+0.1]);

%% 2) Track curvature

f2 = figure(2); hold on
f2.Position = [760.2000 349.8000 560 420.0000];

s_plot = linspace(0,L_track,5000);

K_plot_real = K_fun_real(s_plot);
plot(s_plot,K_plot_real,'r--', 'linewidth', 1)

K_plot = K_fun(s_plot);
plot(s_plot,K_plot,'r-', 'linewidth', 1), hold off, grid on

title('\textbf{Track curvature} $K, \; \tilde{K}$ [1/m]', 'interpreter', 'latex')
xlabel('$s$ [m]', 'interpreter', 'latex')
legend('Real curvature','Relaxed curvature',...
	'NumColumns',2,'location','southoutside')

xlim([0, L_track])
ylim([min(K_plot_real)-0.1, max(K_plot_real)+0.1])

%% 3) Frenet coordinates and altitude

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

n_end = length(u(1,:)); % Time span

f3 = figure(3);
f3.Position = [200.2000 189 1.1192e+03 420];

subplot(1,3,1)
plot(T*(0:1:n_end), x(1,:), 'b-', 'linewidth', 1), grid on
title('$z$ [m]', 'interpreter', 'latex'), xlabel('$t$ [s]', 'interpreter', 'latex')
xlim([0, T*n_end])

subplot(1,3,2)
plot(T*(0:1:n_end), x(11,:), 'r-', 'linewidth', 1), grid on
title('$s$ [m]', 'interpreter', 'latex'), xlabel('$t$ [s]', 'interpreter', 'latex')
xlim([0, T*n_end])

subplot(1,3,3)
plot(T*(0:1:n_end), x(12,:), 'r-', 'linewidth', 1), grid on
ylim([-d_lim, d_lim])
title('$d$ [m]', 'interpreter', 'latex'), xlabel('$t$ [s]', 'interpreter', 'latex')
xlim([0, T*n_end])

%% 4) RPY angles and velocities

f4 = figure(4);
f4.Position = [199.4000 39.4000 1.1192e+03 420];

subplot(2,3,1)
plot(T*(0:1:n_end), x(4,:)*180/pi, 'b-', 'linewidth', 1), grid on
ylim([min(x(4,:)*180/pi)-5, max(x(4,:)*180/pi)+5])
title('$\psi$ [deg]', 'interpreter', 'latex'), xlabel('$t$ [s]', 'interpreter', 'latex')
xlim([0, T*n_end])

subplot(2,3,2)
plot(T*(0:1:n_end), x(3,:)*180/pi, 'b-', 'linewidth', 1), grid on
title('$\theta$ [deg]', 'interpreter', 'latex'), xlabel('$t$ [s]', 'interpreter', 'latex')
xlim([0, T*n_end])

subplot(2,3,3)
plot(T*(0:1:n_end), x(2,:)*180/pi, 'b-', 'linewidth', 1), grid on
title('$\phi$ [deg]', 'interpreter', 'latex'), xlabel('$t$ [s]', 'interpreter', 'latex')
xlim([0, T*n_end])

subplot(2,3,4)
plot(T*(0:1:n_end), x(5,:), 'r-', 'linewidth', 1), grid on
title('$v_x$ [m/s]', 'interpreter', 'latex'), xlabel('$t$ [s]', 'interpreter', 'latex')
xlim([0, T*n_end])

subplot(2,3,5)
plot(T*(0:1:n_end), x(6,:), 'r-', 'linewidth', 1), grid on
title('$v_y$ [m/s]', 'interpreter', 'latex'), xlabel('$t$ [s]', 'interpreter', 'latex')
xlim([0, T*n_end])

subplot(2,3,6)
plot(T*(0:1:n_end), x(7,:), 'r-', 'linewidth', 1), grid on
title('$v_z$ [m/s]', 'interpreter', 'latex'), xlabel('$t$ [s]', 'interpreter', 'latex')
xlim([0, T*n_end])

%% Export images

if print_figures == true
	name_fig_1 = sprintf('mpc_track_%d.jpg',n_save');
	name_fig_2 = sprintf('mpc_curv_%d.jpg',n_save');
	name_fig_3 = sprintf('mpc_frenet_%d.jpg',n_save');
	name_fig_4 = sprintf('mpc_rpy_%d.jpg',n_save');

	exportgraphics(f1,name_fig_1,'Resolution',300);
	exportgraphics(f2,name_fig_2,'Resolution',300);
	exportgraphics(f3,name_fig_3,'Resolution',300);
	exportgraphics(f4,name_fig_4,'Resolution',300);
end

end


























