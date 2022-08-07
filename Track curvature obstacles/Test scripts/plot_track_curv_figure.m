clc, clear variables, close all
interp = @griddedInterpolant;

% ===== BEGIN Main settings =====
print_figures = true;
% ===== END Main settings =====

%% ========== Track ==========

n_track = 3;

if n_track == 1
	track.type = ['s','l','s','l','s'];
	track.radius = [1, 1, 2, 1, 1];
	track.curve = [0, pi, 0, pi, 0];
	d_lim = 0.5;
end

if n_track == 2
	track.type = ['s','l','r','l','s','l','s'];
	track.radius = [1, 1, 1, 1, 2, 1, 1];
	track.curve = [0, pi, pi/2, pi, 0, pi/2, 0];
	d_lim = 0.5;
end

if n_track == 3
	track.type = ['s','l','s','l','s','l','r','r','l','l','s'];
	track.radius = [1,1,4,1,2,1,1,3,1,5,1];
	track.curve = [0,pi/2,0,pi/2,0,pi,pi,pi/2,pi,pi/2,0];
	d_lim = 0.75;
end

% Track length
L_track = sum(track.radius(track.type == 's')) + ...
	sum(track.curve(track.type ~= 's').*track.radius(track.type ~= 's'));

%% ========== Track ==========

f1 = figure(1); hold on

N_plot = 30;

[x_c,y_c] = plot_track(track,0,N_plot);
[x_in,y_in] = plot_track(track,d_lim,N_plot);
[x_out,y_out] = plot_track(track,-d_lim,N_plot);

plot(x_c(1),y_c(1),'k.','markersize',15), axis equal
plot(x_c,y_c,'k-','linewidth',0.5)
plot(x_in,y_in,'k-','linewidth',1)
plot(x_out,y_out,'k-','linewidth',1), hold off, grid on
x_axis_u = max(x_out);
x_axis_l = min(x_out);
y_axis_u = max(y_out);
y_axis_l = min(y_out);

title('\textbf{Track}','interpreter','latex')
xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')

xlim([x_axis_l-0.1, x_axis_u+0.1]);
ylim([y_axis_l-0.1, y_axis_u+0.1]);

%% ========== Curvature ==========

[s_interp, K_interp] = get_curv(track,1e-06);
K_fun_real = interp(s_interp,K_interp,'pchip');

f2 = figure(2);

s_plot = linspace(0,L_track,5000);

K_plot_real = K_fun_real(s_plot);
plot(s_plot,K_plot_real,'r-', 'linewidth', 1), grid on

title('\textbf{Track curvature} $K$ [1/m]', 'interpreter', 'latex')
xlabel('$s$ [m]', 'interpreter', 'latex')
ylim([min(K_plot_real)-0.1, max(K_plot_real)+0.1])
xlim([0, L_track])

%% ========== Export images ==========
if print_figures == true
	name_fig_1 = sprintf('track_%d.jpg',n_track');
	name_fig_2 = sprintf('curv_track_%d.jpg',n_track');

	exportgraphics(f1,name_fig_1,'Resolution',300);
	exportgraphics(f2,name_fig_2,'Resolution',300);
end













