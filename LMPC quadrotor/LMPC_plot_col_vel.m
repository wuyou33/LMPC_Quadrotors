clc, clear variables, close all
load my_colormap

% ===== BEGIN Main settings =====
n_save = 5;
print_figures = true;
% ===== END Main settings =====

data_to_load = sprintf('lmpc_data_%d',n_save);
load(data_to_load);

N_traj = length(x_safe);

%% Trajectories on track with coloured velocity profile

f1 = figure(1); hold on

N_plot_track = 30;

[x_c,y_c] = plot_track(data.track,0,N_plot_track);
[x_in,y_in] = plot_track(data.track,data.d_lim,N_plot_track);
[x_out,y_out] = plot_track(data.track,-data.d_lim,N_plot_track);

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

colormap(my_colormap)

% Trajectories
for i = 1:1:N_traj
	x = x_safe{i};
	v = sqrt(x(5,:).^2+x(6,:).^2);
	v_max(i) = max(v);
	
	[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
	color_line(x_traj,y_traj,v, 'linewidth', 1.5);
end
hold off, grid on

xlim([x_axis_l-0.1, x_axis_u+0.1]);
ylim([y_axis_l-0.1, y_axis_u+0.1]);

color_bar = colorbar;
color_bar.Label.String = '\textbf{Planar velocity} $ v = \sqrt{v_x^2 + v_y^2} \; $ [m/s]';
color_bar.Label.Interpreter = 'latex';

caxis([0, max(v_max)])

%% Export images

if print_figures == true
	name_fig_1 = sprintf('lmpc_col_vel_%d.jpg',n_save');

	exportgraphics(f1,name_fig_1,'Resolution',300);
end



























