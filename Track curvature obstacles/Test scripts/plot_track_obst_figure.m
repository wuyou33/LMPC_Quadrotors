clc, clear variables, close all
interp = @griddedInterpolant;

% ===== BEGIN Main settings =====
n_save = 1;
print_figures = true;
% ===== END Main settings =====

%% ========== Track ==========

track.type = ['s','l','s','l','s'];
track.radius = [1, 1, 2, 1, 1];
track.curve = [0, pi, 0, pi, 0];
d_lim = 0.5;

% Track length
L_track = sum(track.radius(track.type == 's')) + ...
	sum(track.curve(track.type ~= 's').*track.radius(track.type ~= 's'));

%% ========== Obstacles (horizontal - d) ==========

% Inner border
obst_d_i.s = [0, 4.5, 5, 8.5, 9];
obst_d_i.d = [0.5, 0, 0.5, 0.3, 0.5];
% Outer border
obst_d_o.s = [0, 4.5, 5, 8.5, 9];
obst_d_o.d = [-0.5, -0.3, -0.5, -0.1, -0.5];

% Relaxed
c_rel = 0.999;

[s_interp, v_interp] = get_obst(obst_d_i,L_track,c_rel,'d','i');
obst_d_i_fun = interp(s_interp,v_interp,'pchip');

[s_interp, v_interp] = get_obst(obst_d_o,L_track,c_rel,'d','o');
obst_d_o_fun = interp(s_interp,v_interp,'pchip');

% Real
[s_interp, v_interp] = get_obst(obst_d_i,L_track,1e-06,'d','i');
obst_d_i_fun_real = interp(s_interp,v_interp,'pchip');

[s_interp, v_interp] = get_obst(obst_d_o,L_track,1e-06,'d','o');
obst_d_o_fun_real = interp(s_interp,v_interp,'pchip');

%% ========== Obstacles (vertical - z) ==========

% Lower border
obst_z_l.s = [0, 3.5, 4, 8.5, 9];
obst_z_l.z = [0.5, 1.1, 0.5, 0.7, 0.5];
% Upper border
obst_z_u.s = [0, 3.5, 4, 8.5, 9];
obst_z_u.z = [1.5, 1.3, 1.5, 0.9, 1.5];

% Relaxed
c_rel = 0.999;

[s_interp, v_interp] = get_obst(obst_z_l,L_track,c_rel,'z','l');
obst_z_l_fun = interp(s_interp,v_interp,'pchip');

[s_interp, v_interp] = get_obst(obst_z_u,L_track,c_rel,'z','u');
obst_z_u_fun = interp(s_interp,v_interp,'pchip');

% Real
[s_interp, v_interp] = get_obst(obst_z_l,L_track,1e-06,'z','l');
obst_z_l_fun_real = interp(s_interp,v_interp,'pchip');

[s_interp, v_interp] = get_obst(obst_z_u,L_track,1e-06,'z','u');
obst_z_u_fun_real = interp(s_interp,v_interp,'pchip');

%% ========== Track ==========

f1 = figure(1); hold on
f1.Position = [1.3380e+02   3.4900e+02   5.6000e+02   4.2000e+02];

N_plot = 30;

[x_c,y_c] = plot_track(track,0,N_plot);
[x_in,y_in] = plot_track(track,d_lim,N_plot);
[x_out,y_out] = plot_track(track,-d_lim,N_plot);

% Vertical obstacles
for i = 2:1:length(obst_z_l.s)
	[x_obst, y_obst] = plot_on_track(track,...
		[obst_z_l.s(i), obst_z_l.s(i)],[-d_lim, d_lim]);
	v_obst = plot(x_obst,y_obst,'m-','linewidth',1);
end

if length(obst_z_l.s) == 1
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
s_plot = linspace(0,L_track,5000);

d_plot = obst_d_i_fun_real(s_plot);
[x_obst, y_obst] = plot_on_track(track,s_plot,d_plot);
h_obst = plot(x_obst,y_obst,'k-','linewidth',1);
d_plot = obst_d_o_fun_real(s_plot);
[x_obst, y_obst] = plot_on_track(track,s_plot,d_plot);
plot(x_obst,y_obst,'k-','linewidth',1)

if length(obst_d_i.s) == 1
	h_obst = [];
end

hold off, grid on, axis equal

title('\textbf{Track}','interpreter','latex')
xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')

xlim([x_axis_l-0.1, x_axis_u+0.1]);
ylim([y_axis_l-0.1, y_axis_u+0.1]);

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

%% ========== Export images ==========
if print_figures == true
	name_fig_1 = sprintf('track_obst_%d.jpg',n_save);

	exportgraphics(f1,name_fig_1,'Resolution',300);
end













