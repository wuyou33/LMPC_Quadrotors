function [x_mpc, u_mpc] = MPC_obst(x1, c_o, n_o, data, anim_plot)

yalmip('clear')

% ========== Model ==========
nx = 13;
nu = 4;

% ========== MPC data ==========
N = 10; % Prediction horizon

% ========== Yalmip declarations ==========
% ===== Optimization variables =====
% Matrices
A = sdpvar(nx,nx,'full');
B = sdpvar(nx,nu,'full');
c = sdpvar(nx,1,'full');

% States
x = sdpvar(nx*ones(1,N+1),ones(1,N+1));
x1_in = sdpvar(nx,1);
z_r_in = sdpvar(1,1);
d_r_in = sdpvar(1,1);

% Inputs
u = sdpvar(nu*ones(1,N),ones(1,N));

% Slack variable
e1 = sdpvar(1,1); % Horizontal bounds
e2 = sdpvar(1,1); % Vertical bounds

% ===== Cost function =====

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

cost = 0;

x_r = [z_r_in, 0, 0, pi/4, 0, 0, 0, 0, 0, 0, data.L_track, d_r_in, 0]';

Q = diag([100, 1, 1, 10, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 1, 1e03, 0]);
R = diag([0.01, 0.01, 0.01, 0.01]);

Qd = (1/data.T^2)*diag([0, 0, 0, 0, 100, 100, 10, 0.01, 0.01, 100, 0, 0, 0]);
Rd = (1/data.T^2)*diag([0.01, 0.01, 0.01, 0.01]);

for k = 1:1:N
	cost = cost + (x{k}-x_r)'*Q*(x{k}-x_r) + u{k}'*R*u{k};
end

for k = 2:1:N
	cost = cost + (x{k}-x{k-1})'*Qd*(x{k}-x{k-1}) + ...
		(u{k}-u{k-1})'*Rd*(u{k}-u{k-1});
end

cost = cost + 1e03*e1^2 + 1e03*e2^2;

% ===== Constraints =====
constr = [];

for k = 1:1:N
	constr = [constr;
		x{k+1} == A*x{k} + B*u{k} + c];
	
	constr = [constr;
		-data.d_lim - e1 <= x{k}(12) <= data.d_lim + e1;
		0 - e2 <= x{k}(1) <= 1.5 + e2;
		-0.5/sqrt(2) <= x{k}(5) <= 0.5/sqrt(2);
		-0.5/sqrt(2) <= x{k}(6) <= 0.5/sqrt(2);
		-0.5 <= x{k}(7) <= 0.5];
end

constr = [constr;
		x{1} == x1_in;
		e1 >= 0;
		e2 >= 0];

% ===== Optimizer object =====
params_in = {x1_in, A, B, c, z_r_in, d_r_in};
sol_out = u{1};

options = sdpsettings('verbose',0,'solver','quadprog');
mpc = optimizer(constr,cost,options,params_in,sol_out);

% ========== Opt. prob. resol. and control ==========

x_mpc = x1;
u_mpc = [];

u1 = [0; 0; 0; 0];

% ===== BEGIN Animated plot (1) =====
f1 = figure(1);
f1.Position = [0.1354    0.2642    1.2512    0.4978]*10^3;

% Track
subplot(1,2,1), hold on, grid on, axis equal
N_plot = 30;

[x_c,y_c] = plot_track(data.track,0,N_plot);
[x_in,y_in] = plot_track(data.track,data.d_lim,N_plot);
[x_out,y_out] = plot_track(data.track,-data.d_lim,N_plot);
x_axis_u = max(x_out);
x_axis_l = min(x_out);
y_axis_u = max(y_out);
y_axis_l = min(y_out);

for i = 2:1:length(data.obst_z_l.s)
	[x_obst, y_obst] = plot_on_track(data.track,...
		[data.obst_z_l.s(i), data.obst_z_l.s(i)],[-data.d_lim, data.d_lim]);
	plot(x_obst,y_obst,'m-','linewidth',1);
end

plot(x_c(1),y_c(1),'k.','markersize',15)
plot(x_c,y_c,'k-','linewidth',0.5)
plot(x_in,y_in,'-','color',[0.7 0.7 0.7],'linewidth',1)
plot(x_out,y_out,'-','color',[0.7 0.7 0.7],'linewidth',1)

s_plot = linspace(0,data.L_track,5000);

d_plot = data.obst_d_i_fun_real(s_plot);
[x_obst, y_obst] = plot_on_track(data.track,s_plot,d_plot);
plot(x_obst,y_obst,'k-','linewidth',1)
d_plot = data.obst_d_o_fun_real(s_plot);
[x_obst, y_obst] = plot_on_track(data.track,s_plot,d_plot);
plot(x_obst,y_obst,'k-','linewidth',1)

[q_loc_x, q_loc_y] = plot_on_track(data.track,x1(11),x1(12));
q_loc = plot(q_loc_x,q_loc_y,'r.','markersize',15);

xlim([x_axis_l-0.1, x_axis_u+0.1]);
ylim([y_axis_l-0.1, y_axis_u+0.1]);

title('\textbf{Track}','interpreter','latex')

% Altitude
subplot(1,2,2), hold on, grid on

plot([0,data.L_track],[0.5, 0.5],'-','color',[0.7 0.7 0.7],'linewidth',1)
plot([0,data.L_track],[1.5, 1.5],'-','color',[0.7 0.7 0.7],'linewidth',1)

z_plot = data.obst_z_u_fun_real(s_plot);
plot(s_plot,z_plot,'k-', 'linewidth', 1)
z_plot = data.obst_z_l_fun_real(s_plot);
plot(s_plot,z_plot,'k-', 'linewidth', 1)

q_alt = plot(x1(11),x1(1),'r.','markersize',15);

xlim([-inf, data.L_track])
title('\textbf{Altitude} $z$','interpreter','latex')

step_disp = 1;

pause
% ===== END Animated plot (1) =====

while x1(11) < data.L_track % Exit condition
	
	K1 = data.K_fun(x1(11)); % Relaxed curvature
	
	% Update ATV model
	A = quadrot_A(x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),...
		x1(7),x1(8),x1(9),x1(10),x1(11),x1(12),x1(13),...
		u1(1),u1(2),u1(3),u1(4),K1,data.T);
	
	B = quadrot_B(x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),...
		x1(7),x1(8),x1(9),x1(10),x1(11),x1(12),x1(13),...
		u1(1),u1(2),u1(3),u1(4),K1,data.T);
	
	c = quadrot_nl(x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),...
		x1(7),x1(8),x1(9),x1(10),x1(11),x1(12),x1(13),...
		u1(1),u1(2),u1(3),u1(4),K1,data.T) - (A*x1 + B*u1);
	
	% ===== BEGIN MPC =====
	% Obstacles
	z_l = data.obst_z_l_fun(x1(11));
	z_u = data.obst_z_u_fun(x1(11));
	d_i = data.obst_d_i_fun(x1(11));
	d_o = data.obst_d_o_fun(x1(11));
	
	z_r = (z_l+z_u)/2;
	
	d_r = c_o*(d_i-d_o)/2*sin(n_o*2*pi*x1(11)/data.L_track) + (d_i+d_o)/2;
	
	u1 = mpc({x1,A,B,c,z_r,d_r}); % Get optimal control input
	
	% Apply optimal control input and get next states
	x2 = quadrot_nl(x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),...
		x1(7),x1(8),x1(9),x1(10),x1(11),x1(12),x1(13),...
		u1(1),u1(2),u1(3),u1(4),data.K_fun_real(x1(11)),data.T);
	
	% Store optimal states and inputs
	x_mpc = [x_mpc, x2];
	u_mpc = [u_mpc, u1];
	% ===== END MPC =====
	
	% ===== BEGIN Animated plot (2) =====
	subplot(1,2,1)
	
	delete(q_loc);
	
	[x_traj_1, y_traj_1] = plot_on_track(data.track,x1(11),x1(12));
	[x_traj_2, y_traj_2] = plot_on_track(data.track,x2(11),x2(12));
	plot([x_traj_1, x_traj_2],[y_traj_1, y_traj_2],'g-','linewidth',1)
	
	[q_loc_x, q_loc_y] = plot_on_track(data.track,x2(11),x2(12));
	q_loc = plot(q_loc_x,q_loc_y,'r.','markersize',15);
	
	if anim_plot == true
		drawnow
	end
	
	subplot(1,2,2)
	
	delete(q_alt);
	
	plot([x1(11), x2(11)], [x1(1), x2(1)], 'g-', 'linewidth', 1)
	
	q_alt = plot(x2(11),x2(1),'r.','markersize',15);
	
	if anim_plot == true
		drawnow
	end
	
	clc
	fprintf('Iteration 0\n');
	fprintf('Time = %1.2f s\n',step_disp*data.T);
	step_disp = step_disp + 1;
	% ===== END Animated plot (2) =====
	
	% Update initial states
	x1 = x2;
	
end

delete(q_loc);
delete(q_alt);

end




















