clc, clear variables, close all
load my_colormap

% ===== BEGIN Main settings =====
n_save = 1;
print_figures = true;
% ===== END Main settings =====

data_to_load = sprintf('lmpc_obst_data_%d',n_save);
load(data_to_load);

N_traj = length(x_safe);

%% Trajectories on track with coloured velocity profile

f1 = figure(1); hold on

N_plot = 30;

[x_c,y_c] = plot_track(data.track,0,N_plot);
[x_in,y_in] = plot_track(data.track,data.d_lim,N_plot);
[x_out,y_out] = plot_track(data.track,-data.d_lim,N_plot);

% Vertical obstacles

for i = 2:1:length(data.obst_z_l.s)
	[x_obst, y_obst] = plot_on_track(data.track,...
		[data.obst_z_l.s(i), data.obst_z_l.s(i)],[-data.d_lim, data.d_lim]);
	v_obst = plot(x_obst,y_obst,'m-','linewidth',1);
end

if length(data.obst_z_l.s) == 1
	v_obst = [];
end

% Track
plot(x_c(1),y_c(1),'k.','markersize',15)
plot(x_c,y_c,'k-','linewidth',0.5)
plot(x_in,y_in,'-','color',[0.7 0.7 0.7],'linewidth',1)
plot(x_out,y_out,'-','color',[0.7 0.7 0.7],'linewidth',1)
x_axis_u = max(x_out);
x_axis_l = min(x_out);
y_axis_u = max(y_out);
y_axis_l = min(y_out);

% Horizontal obstacles
s_plot = linspace(0,data.L_track,5000);

d_plot = data.obst_d_i_fun_real(s_plot);
[x_obst, y_obst] = plot_on_track(data.track,s_plot,d_plot);
h_obst = plot(x_obst,y_obst,'k-','linewidth',1);
d_plot = data.obst_d_o_fun_real(s_plot);
[x_obst, y_obst] = plot_on_track(data.track,s_plot,d_plot);
plot(x_obst,y_obst,'k-','linewidth',1)

if length(data.obst_d_i.s) == 1
	h_obst = [];
end

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
hold off, grid on, axis equal

xlim([x_axis_l-0.1, x_axis_u+0.1]);
ylim([y_axis_l-0.1, y_axis_u+0.1]);

color_bar = colorbar;
color_bar.Label.String = '\textbf{Planar velocity} $ v = \sqrt{v_x^2 + v_y^2} \; $ [m/s]';
color_bar.Label.Interpreter = 'latex';

caxis([0, max(v_max)])

if isempty(v_obst)
	if isempty(h_obst)
	else
		legend(h_obst,...
			'Track/Horiz. obstacles',...
			'NumColumns',3,'location','southoutside')
	end
else
	if isempty(h_obst)
		legend(v_obst,...
			'Vert. obstacles',...
			'NumColumns',3,'location','southoutside')
	else
		legend([h_obst, v_obst],...
			'Track/Horiz. obstacles','Vert. obstacles',...
			'NumColumns',3,'location','southoutside')
	end
end

%% Export images

if print_figures == true
	name_fig_1 = sprintf('lmpc_obst_col_vel_%d.jpg',n_save');

	exportgraphics(f1,name_fig_1,'Resolution',300);
end



























