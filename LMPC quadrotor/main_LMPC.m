clc, clear variables, close all
interp = @griddedInterpolant;
rng('default')

% ===== BEGIN Main settings =====
n_track = 1; % Select track
rep = true; % Repetitive LMPC
anim_plot = true; % Animated plot

n_save = 9;
print_figures = false;
% ===== END Main settings =====

%% ========== Model ==========
T = 0.2; % Discrete time interval

%% ========== Track ==========

if n_track == 1
	track.type = ['s','l','s','l','s'];
	track.radius = [1, 1, 2, 1, 1];
	track.curve = [0, pi, 0, pi, 0];
	d_lim = 0.5;
	
	% Track-dependant values
	if rep == true
		Q_s = 15; Q_d = 1e05; Qd_v = 5e04; Qd_vz = 1e04;
		d_r_coef = 0.3/d_lim;
		c_o = 0.5; n_o = 4;
% 		c_o = 0.9; n_o = 1;
		n_iter = 7;
	else
		Q_s = 15; Q_d = 1e05; Qd_v = 5e04; Qd_vz = 1e04;
		d_r_coef = 0.3/d_lim;
		c_o = 0.9; n_o = 1;
		n_iter = 5;
	end
end

if n_track == 2
	track.type = ['s','l','r','l','s','l','s'];
	track.radius = [1, 1, 1, 1, 2, 1, 1];
	track.curve = [0, pi, pi/2, pi, 0, pi/2, 0];
	d_lim = 0.5;
	
	% Track-dependant values
	Q_s = 10; Q_d = 1e05; Qd_v = 5e04; Qd_vz = 1e04;
	d_r_coef = 0.3/d_lim;
	c_o = 0.5; n_o = 4;
	n_iter = 6;
end

if n_track == 3
	track.type = ['s','l','s','l','s','l','r','r','l','l','s'];
	track.radius = [1,1,4,1,2,1,1,3,1,5,1];
	track.curve = [0,pi/2,0,pi/2,0,pi,pi,pi/2,pi,pi/2,0];
	d_lim = 0.75;
	
	% Track-dependant values
	Q_s = 1; Q_d = 1e05; Qd_v = 5e04; Qd_vz = 1e04;
	d_r_coef = 0.3/d_lim;
	c_o = 0.5; n_o = 4;
	n_iter = 5;
end

% Track length
L_track = sum(track.radius(track.type == 's')) + ...
	sum(track.curve(track.type ~= 's').*track.radius(track.type ~= 's'));

%% ========== Curvature ==========
% Relaxed
c_rel = 0.1;
[s_interp, K_interp] = get_curv(track,c_rel);
K_fun = interp(s_interp,K_interp,'pchip');

% Real
[s_interp, K_interp] = get_curv(track,1e-06);
K_fun_real = interp(s_interp,K_interp,'pchip');

%% ========== Others ==========
z_r = 1; % Reference altitude

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

data.z_r = z_r;

%% =============== First trajectory (SS init.) - MPC ===============

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

x1 = [0, 0, 0, deg2rad(45), 0, 0, 0, 0, 0, 0, 0, 0, 0]';

[x_mpc, u_mpc] = MPC(x1, [c_o, n_o], data, rep, anim_plot);

%% =============== LMPC ===============
N_lmpc = 10;

if rep == true
	x1 = x_mpc(:,end);
	x1(11) = x1(11) - L_track;
	x1(13) = x1(13) - 2*pi;
end

[x_safe, u_safe, Q_safe] = LMPC(x1, x_mpc, u_mpc, N_lmpc, n_iter, data, rep, anim_plot);

% Save data externally
data_to_save = sprintf('lmpc_data_%d',n_save);
save(data_to_save,'x_safe','u_safe','Q_safe','data','rep');

%% =============== Plots ===============
close all
pause
LMPC_plot_results_fun(n_save,print_figures)


























