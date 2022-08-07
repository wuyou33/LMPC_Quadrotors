clc, clear variables, close all
interp = @griddedInterpolant;

% ===== BEGIN Main settings =====
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

%% ========== Plot ==========

% ===== Obstacles functions (horizontal - d) =====
f1 = figure(1);
f1.Position = [0.1042    0.3420    1.3280    0.4200]*10^3;

s_plot = linspace(0,L_track,5000);

% Outer border
subplot(1,2,1)
v_plot = obst_d_o_fun_real(s_plot);
plot(s_plot,v_plot,'r--', 'linewidth', 1), hold on
v_plot = obst_d_o_fun(s_plot);
plot(s_plot,v_plot,'r-', 'linewidth', 1), hold off, grid on

title('\textbf{Outer border} $d_{o,l}, \; \tilde{d}_{o,l}$ [m]', 'interpreter', 'latex')
xlabel('$s$ [m]', 'interpreter', 'latex')
legend('Real obstacle','Relaxed obstacle','Location','southoutside','NumColumns',2)
ylim([min(v_plot)-0.1, max(v_plot)+0.1])
xlim([0, L_track])

% Inner border
subplot(1,2,2)
v_plot = obst_d_i_fun_real(s_plot);
plot(s_plot,v_plot,'r--', 'linewidth', 1), hold on
v_plot = obst_d_i_fun(s_plot);
plot(s_plot,v_plot,'r-', 'linewidth', 1), hold off, grid on

title('\textbf{Inner border} $d_{o,u}, \; \tilde{d}_{o,u}$ [m]', 'interpreter', 'latex')
xlabel('$s$ [m]', 'interpreter', 'latex')
legend('Real obstacle','Relaxed obstacle','Location','southoutside','NumColumns',2)
ylim([min(v_plot)-0.1, max(v_plot)+0.1])
xlim([0, L_track])

% ===== Obstacles functions (vertical - z) =====
f2 = figure(2);
f2.Position = [0.1042    0.3420    1.3280    0.4200]*10^3;

% Lower border
subplot(1,2,1)
v_plot = obst_z_l_fun_real(s_plot);
plot(s_plot,v_plot,'r--', 'linewidth', 1), hold on
v_plot = obst_z_l_fun(s_plot);
plot(s_plot,v_plot,'r-', 'linewidth', 1), hold off, grid on

title('\textbf{Lower border} $z_{o,l}, \; \tilde{z}_{o,l}$ [m]', 'interpreter', 'latex')
xlabel('$s$ [m]', 'interpreter', 'latex')
legend('Real obstacle','Relaxed obstacle','Location','southoutside','NumColumns',2)
ylim([min(v_plot)-0.1, max(v_plot)+0.1])
xlim([0, L_track])

% Upper border
subplot(1,2,2)
v_plot = obst_z_u_fun_real(s_plot);
plot(s_plot,v_plot,'r--', 'linewidth', 1), hold on
v_plot = obst_z_u_fun(s_plot);
plot(s_plot,v_plot,'r-', 'linewidth', 1), hold off, grid on

title('\textbf{Upper border} $z_{o,u}, \; \tilde{z}_{o,u}$ [m]', 'interpreter', 'latex')
xlabel('$s$ [m]', 'interpreter', 'latex')
legend('Real obstacle','Relaxed obstacle','Location','southoutside','NumColumns',2)
ylim([min(v_plot)-0.1, max(v_plot)+0.1])
xlim([0, L_track])

%% ========== Export images ==========
if print_figures == true
	exportgraphics(f1,'obst_hor_d.jpg','Resolution',300);
	exportgraphics(f2,'obst_vert_z.jpg','Resolution',300);
end















