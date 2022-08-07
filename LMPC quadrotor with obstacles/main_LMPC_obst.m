clc, clear variables, close all
interp = @griddedInterpolant;
rng('default')

% ===== BEGIN Main settings =====
n_obst = 1; % Select obstacles
anim_plot = true; % Animated plot

n_save = 2;
print_figures = false;
% ===== END Main settings =====

%% ========== Model ==========
T = 0.2; % Discrete time interval

%% ========== Track ==========

track.type = ['s','l','s','l','s'];
track.radius = [1, 1, 2, 1, 1];
track.curve = [0, pi, 0, pi, 0];
d_lim = 0.5;

% Track length
L_track = sum(track.radius(track.type == 's')) + ...
	sum(track.curve(track.type ~= 's').*track.radius(track.type ~= 's'));

% Track-dependant values
Q_s = 20; Q_d = 1e05; Qd_v = 5e04; Qd_vz = 1e03;
d_r_coef = 0.3/d_lim;
c_o = 0.5; n_o = 4;
n_iter = 7;

%% ========== Curvature ==========
% Relaxed
K_rel = 0.1;
[s_interp, K_interp] = get_curv(track,K_rel);
K_fun = interp(s_interp,K_interp,'pchip');

% Real
[s_interp, K_interp] = get_curv(track,1e-06);
K_fun_real = interp(s_interp,K_interp,'pchip');

%% ========== Obstacles (horizontal - d) ==========

if n_obst == 1
	% Inner border
	obst_d_i.s = [0, 4.5, 5, 8.5, 9];
	obst_d_i.d = [0.5, 0, 0.5, 0.3, 0.5];
	% Outer border
	obst_d_o.s = [0, 4.5, 5, 8.5, 9];
	obst_d_o.d = [-0.5, -0.3, -0.5, -0.1, -0.5];
end
if n_obst == 2
	% Inner border
	obst_d_i.s = [0];
	obst_d_i.d = [0.5];
	% Outer border
	obst_d_o.s = [0];
	obst_d_o.d = [-0.5];
end

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

if n_obst == 1
	% Lower border
	obst_z_l.s = [0, 3.5, 4, 8.5, 9];
	obst_z_l.z = [0.5, 1.1, 0.5, 0.7, 0.5];
	% Upper border
	obst_z_u.s = [0, 3.5, 4, 8.5, 9];
	obst_z_u.z = [1.5, 1.3, 1.5, 0.9, 1.5];
end
if n_obst == 2
	% Lower border
	obst_z_l.s = [0];
	obst_z_l.z = [0.5];
	% Upper border
	obst_z_u.s = [0];
	obst_z_u.z = [1.5];
end

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

%% ========== Save relevant data ("data" variable) ==========
data.T = T;

data.track = track;
data.d_lim = d_lim;
data.L_track = L_track;
data.Q_s = Q_s;
data.Q_d = Q_d;
data.Qd_v = Qd_v;
data.Qd_vz = Qd_vz;
data.d_r_coef = d_r_coef;

data.K_fun = K_fun;
data.K_fun_real = K_fun_real;

data.obst_d_i_fun = obst_d_i_fun;
data.obst_d_o_fun = obst_d_o_fun;
data.obst_z_l_fun = obst_z_l_fun;
data.obst_z_u_fun = obst_z_u_fun;

data.obst_d_i_fun_real = obst_d_i_fun_real;
data.obst_d_o_fun_real = obst_d_o_fun_real;
data.obst_z_l_fun_real = obst_z_l_fun_real;
data.obst_z_u_fun_real = obst_z_u_fun_real;

data.obst_d_i = obst_d_i;
data.obst_z_l = obst_z_l;

%% =============== First trajectory (SS init.) - MPC ===============

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

x1 = [0, 0, 0, deg2rad(45), 0, 0, 0, 0, 0, 0, 0, 0, 0]';

[x_mpc, u_mpc] = MPC_obst(x1, c_o, n_o, data, anim_plot);

%% =============== LMPC ===============
N_lmpc = 10;

x1 = x_mpc(:,end);
x1(11) = x1(11) - L_track;
x1(13) = x1(13) - 2*pi;

[x_safe, u_safe, Q_safe] = LMPC_obst(x1, x_mpc, u_mpc, N_lmpc, n_iter, data, anim_plot);

% Save data externally
data_to_save = sprintf('lmpc_obst_data_%d',n_save);
save(data_to_save,'x_safe','u_safe','Q_safe','data');

%% =============== Plots ===============
close all
pause
LMPC_obst_plot_results_fun(n_save,print_figures)


























