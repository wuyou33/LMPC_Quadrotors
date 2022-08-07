function LMPC_obst_plot_results_fun(n_save, print_figures)

data_to_load = sprintf('lmpc_obst_data_%d',n_save);
load(data_to_load);

N_traj = length(x_safe);

for i=1:1:N_traj
	Q_traj(i) = Q_safe{i}(1);
end

N_traj_vec = 1:1:N_traj;
min_cost_traj = N_traj_vec(Q_traj == min(Q_traj));
min_cost_traj = min_cost_traj(end);

%% 1) Trajectories on track

f1 = figure(1); hold on
f1.Position = [1.3380e+02   3.4900e+02   5.6000e+02   4.2000e+02];

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

% Trajectories

x = x_safe{1};
[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
traj_p(1) = plot(x_traj,y_traj,'g-','linewidth',1);

for i = 2:1:N_traj-1
	x = x_safe{i};
	[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
	plot(x_traj,y_traj,'b-','linewidth',1)
end

x = x_safe{N_traj};
[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
traj_p(2) = plot(x_traj,y_traj,'b-','linewidth',1);

x = x_safe{min_cost_traj};
[x_traj, y_traj] = plot_on_track(data.track,x(11,:),x(12,:));
traj_p(3) = plot(x_traj,y_traj,'r-','linewidth',2); hold off, grid on, axis equal

if isempty(v_obst)
	if isempty(h_obst)
		legend(traj_p,'MPC','LMPC','Best',...
			'NumColumns',3,'location','southoutside')
	else
		legend([traj_p, h_obst],'MPC','LMPC','Best',...
			'Track/Horiz. obstacles',...
			'NumColumns',3,'location','southoutside')
	end
else
	if isempty(h_obst)
		legend([traj_p, v_obst],'MPC','LMPC','Best',...
			'Vert. obstacles',...
			'NumColumns',3,'location','southoutside')
	else
		legend([traj_p, h_obst, v_obst],'MPC','LMPC','Best',...
			'Track/Horiz. obstacles','Vert. obstacles',...
			'NumColumns',3,'location','southoutside')
	end
end

xlim([x_axis_l-0.1, x_axis_u+0.1]);
ylim([y_axis_l-0.1, y_axis_u+0.1]);

%% 2) Altitude

f2 = figure(2); hold on
f2.Position = [5.3540e+02   3.4900e+02   5.6000e+02   4.2000e+02];

plot([0,data.L_track],[0.5, 0.5],'-','color',[0.7 0.7 0.7],'linewidth',1)
plot([0,data.L_track],[1.5, 1.5],'-','color',[0.7 0.7 0.7],'linewidth',1)

% Vertical ostacles

v_plot = data.obst_z_u_fun_real(s_plot);
v_obst = plot(s_plot,v_plot,'k-', 'linewidth', 1);
v_plot = data.obst_z_l_fun_real(s_plot);
plot(s_plot,v_plot,'k-', 'linewidth', 1)

if length(data.obst_z_l.s) == 1
	v_obst = [];
end

% Trajectories

x = x_safe{1};
traj_p(1) = plot(x(11,:), x(1,:), 'g-', 'linewidth', 1);

for i = 2:1:N_traj-1
	x = x_safe{i};
	plot(x(11,:), x(1,:), 'b-', 'linewidth', 1)
end

x = x_safe{N_traj};
traj_p(2) = plot(x(11,:), x(1,:), 'b-', 'linewidth', 1);
	
x = x_safe{min_cost_traj};
traj_p(3) = plot(x(11,:), x(1,:), 'r-', 'linewidth', 2); hold off, grid on

if isempty(v_obst)
	legend(traj_p,'MPC','LMPC','Best',...
		'NumColumns',3,'location','southoutside')
else
	legend([traj_p, v_obst],'MPC','LMPC','Best',...
		'Vert. obstacles',...
		'NumColumns',3,'location','southoutside')
end

title('\textbf{Altitude} $z$ [m]','interpreter','latex')
xlabel('$s$ [m]','interpreter','latex')
xlim([0, data.L_track])

%% 3) Velocity profiles

f3 = figure(3); hold on
f3.Position = [1.3300e+02   6.8200e+01   8.3280e+02   4.2000e+02];

for i=1:1:N_traj
	
	if(i == min_cost_traj)
		plot(x_safe{i}(11,:), sqrt(x_safe{i}(5,:).^2 + x_safe{i}(6,:).^2), ...
			'r-', 'linewidth', 2)
	else
		plot(x_safe{i}(11,:), sqrt(x_safe{i}(5,:).^2 + x_safe{i}(6,:).^2))
	end
	
end

leg = cell(N_traj,1);
for i=1:1:N_traj
	leg_entry = num2str(i-1);
	leg{i} = leg_entry;
end
leg_obj = legend(leg,'NumColumns',1,'location','eastoutside');
title(leg_obj,'\textbf{Iteration}','interpreter','latex')

hold off, grid on

title('\textbf{Planar velocity} $ v = \sqrt{v_x^2 + v_y^2} \; $ [m/s]',...
	'interpreter','latex')
xlabel('$s$ [m]','interpreter','latex')
xlim([0, data.L_track])

%% 4) Iteration cost

f4 = figure(4); hold on
f4.Position = [9.6420e+02   3.4660e+02   5.6000e+02   4.2000e+02];

plot(0:1:N_traj-1, Q_traj, 'b.-','markersize',10)
for i = 1:1:N_traj
	
	data_val = sprintf('%1.6f',Q_traj(i)/10^4);
	
	if i == 1
		horiz_align = 'left';
		vert_align = 'top';
	else
		if i == N_traj
			horiz_align = 'right';
			vert_align = 'top';
		else
			horiz_align = 'center';
			vert_align = 'bottom';
		end
	end
	
	text(i-1,Q_traj(i),data_val,...
		'VerticalAlignment',vert_align,'HorizontalAlignment',horiz_align,...
		'FontSize',9);
end

hold off

title('\textbf{Iteration cost}','interpreter','latex')
xlabel('Iteration','interpreter','latex')
xticks(0:1:N_traj-1)
set(gca,'YMinorTick','on','YMinorGrid','on'), grid on

%% 5) Lap time

f5 = figure(5); hold on
f5.Position = [9.6420e+02   6.9000e+01   5.6000e+02   4.2000e+02];

for i=1:1:N_traj
	lap_time(i) = length(x_safe{i,1})*data.T;
end

plot(0:1:N_traj-1, lap_time, 'b.-','markersize',10)
for i = 1:1:N_traj
	
	data_val = sprintf('%1.2f',lap_time(i));
	
	if i == 1
		horiz_align = 'left';
		vert_align = 'top';
	else
		if i == N_traj
			horiz_align = 'right';
			vert_align = 'bottom';
		else
			horiz_align = 'center';
			vert_align = 'bottom';
		end
	end
	
	text(i-1,lap_time(i),data_val,...
		'VerticalAlignment',vert_align,'HorizontalAlignment',horiz_align,...
		'FontSize',9);
end

hold off

title('\textbf{Lap time} [s]','interpreter','latex')
xlabel('Iteration','interpreter','latex')
xticks(0:1:N_traj-1)
set(gca,'YMinorTick','on','YMinorGrid','on'), grid on

%% Print iteration cost and lap time on terminal

clc
fprintf('Iteration cost:\n');
for j=1:1:N_traj
	fprintf('%d\t%1.10e\n', j-1, Q_safe{j,1}(1));
end

fprintf('Lap time:\n');
for j=1:1:N_traj
	fprintf('%d\t%1.2f s\n', j-1, length(x_safe{j,1})*data.T);
end

%% Export images

if print_figures == true
	name_fig_1 = sprintf('lmpc_obst_track_%d.jpg',n_save');
	name_fig_2 = sprintf('lmpc_obst_altitude_%d.jpg',n_save');
	name_fig_3 = sprintf('lmpc_obst_velocity_%d.jpg',n_save');
	name_fig_4 = sprintf('lmpc_obst_iter_cost_%d.jpg',n_save');
	name_fig_5 = sprintf('lmpc_obst_steps_%d.jpg',n_save');

	exportgraphics(f1,name_fig_1,'Resolution',300);
	exportgraphics(f2,name_fig_2,'Resolution',300);
	exportgraphics(f3,name_fig_3,'Resolution',300);
	exportgraphics(f4,name_fig_4,'Resolution',300);
	exportgraphics(f5,name_fig_5,'Resolution',300);
end

end


























