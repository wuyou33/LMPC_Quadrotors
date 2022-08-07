clc, clear variables, close all
interp = @griddedInterpolant;
rng('default')

% ===== BEGIN Main settings =====
n_track = 3; % Select track
const_d_r = false; % MPC for constant d_r tracking
anim_plot = false; % Animated plot

n_save = 6;
print_figures = true;
% ===== END Main settings =====

%% ========== Model ==========
T = 0.2; % Discrete time interval

%% ========== Track ==========

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
% Relaxed
c_rel = 0.1;
[s_interp, K_interp] = get_curv(track,c_rel);
K_fun = interp(s_interp,K_interp,'pchip');

% Real
[s_interp, K_interp] = get_curv(track,1e-06);
K_fun_real = interp(s_interp,K_interp,'pchip');

%% ========== Other variables ==========
z_r = 1; % Reference altitude
d_r = 0.375; % Reference lateral distance
c_o = -0.5; n_o = 4; % Trajectory oscillations

%% ========== Save relevant data ("data" variable) ==========
data.T = T;
data.track = track;
data.d_lim = d_lim;
data.L_track = L_track;

data.K_fun = K_fun;
data.K_fun_real = K_fun_real;

data.z_r = z_r;

%% =============== MPC ===============

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

if const_d_r == true
	x1 = [0, 0, 0, deg2rad(45), 0, 0, 0, 0, 0, 0, 0, d_r, 0]';
	[x_mpc, u_mpc] = MPC(x1, d_r, data, false, false);
else
	x1 = [0, 0, 0, deg2rad(45), 0, 0, 0, 0, 0, 0, 0, 0, 0]';
	[x_mpc, u_mpc] = MPC(x1, [c_o, n_o], data, false, false);
end

% Save data externally
data_to_save = sprintf('mpc_data_%d',n_save);
save(data_to_save,'x_mpc','u_mpc','data');

%% =============== Plots ===============
close all
MPC_plot_results_fun(n_save,print_figures)


























