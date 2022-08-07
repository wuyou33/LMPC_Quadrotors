clc, clear variables, close all
interp = @griddedInterpolant;

% ===== BEGIN Main settings =====
print_figures = true;
% ===== END Main settings =====

%% ========== Track ==========

n_track = 2;

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

%% ========== Curvature ==========

c_rel = 0.3;
[s_interp, K_interp] = get_curv(track,c_rel);
K_fun = interp(s_interp,K_interp,'pchip');

[s_interp, K_interp] = get_curv(track,1e-06);
K_fun_real = interp(s_interp,K_interp,'pchip');

f1 = figure(1);

s_plot = linspace(0,L_track,5000);

K_plot_real = K_fun_real(s_plot);
plot(s_plot,K_plot_real,'r--', 'linewidth', 1), hold on

K_plot = K_fun(s_plot);
plot(s_plot,K_plot,'r-', 'linewidth', 1), hold off, grid on

title('\textbf{Track curvature} $K, \; \tilde{K}$ [1/m]', 'interpreter', 'latex')
xlabel('$s$ [m]', 'interpreter', 'latex')
legend('Real curvature','Relaxed curvature',...
	'NumColumns',2,'location','southoutside')
ylim([min(K_plot_real)-0.1, max(K_plot_real)+0.1])

%% ========== Export images ==========
if print_figures == true
	exportgraphics(f1,'curv_track_2_rel.jpg','Resolution',300);
end















