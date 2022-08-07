function u1 = LMPC_OP_obst(x1, A, B, c, A_ch, b_ch, a_cf, b_cf, N, data)

yalmip('clear')

%% ========== Model ==========
nx = 13;
nu = 4;

%% ========== Yalmip declarations ==========
% ===== Optimization variables =====
% States
x = sdpvar(nx*ones(1,N+1),ones(1,N+1));
x1_in = sdpvar(nx,1);

% Inputs
u = sdpvar(nu*ones(1,N),ones(1,N));

% Slack variables
e1 = sdpvar(1,1); % Horizontal bounds/obstacles
e2 = sdpvar(1,1); % Vertical bounds/obstacles
e3 = sdpvar(1,1); % Terminal constraint

% Auxiliary variables
q = sdpvar(1,1); % Terminal cost

% ===== Obstacles =====

z_l = data.obst_z_l_fun(x1(11));
z_u = data.obst_z_u_fun(x1(11));
d_i = data.obst_d_i_fun(x1(11));
d_o = data.obst_d_o_fun(x1(11));

% ===== Cost function =====

cost = 0;

K_sign = sign(data.K_fun_real(x1(11)));

if K_sign == 0 || K_sign == 1
	d_r = d_i * data.d_r_coef;
else
	d_r = d_o * data.d_r_coef;
end

z_r = (z_l+z_u)/2;

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

x_r = [z_r, 0, 0, pi/4, 0, 0, 0, 0, 0, 0, data.L_track*1.2, d_r, 0]';

Q = diag([1e05, 0, 0, 1, 0, 0, 0, 0, 0, 0, data.Q_s, data.Q_d, 0]);

R = diag([0.01, 0.01, 0.01, 0.01]);

Qd = (1/data.T^2)*...
	diag([10, 0.01, 0.01, 10, data.Qd_v, data.Qd_v, data.Qd_vz, 0.01, 0.01, 1e03, 10, 0.01, 0]);
Rd = (1/data.T^2)*diag([0.01, 0.01, 0.01, 0.01]);

for k = 1:1:N
	cost = cost + (x{k}-x_r)'*Q*(x{k}-x_r) + u{k}'*R*u{k};
end

for k = 2:1:N
	cost = cost + (x{k}-x{k-1})'*Qd*(x{k}-x{k-1}) + (u{k}-u{k-1})'*Rd*(u{k}-u{k-1});
end

cost = cost + q; % Terminal cost

cost = cost + 1e08*e1^2 + 1e08*e2^2 + 1e03*e3^2;

% ===== Constraints =====
constr = [];

for k = 1:1:N
	constr = [constr;
		x{k+1} == A*x{k} + B*u{k} + c];
	
	constr = [constr;
		d_o - e1 <= x{k}(12) <= d_i + e1];
	
	constr = [constr;
		z_l - e2 <= x{k}(1) <= z_u + e2];
	
end

constr = [constr;
	x{1} == x1_in];

% Terminal constraint
constr = [constr;
	A_ch*x{N+1}([11,12]) <= b_ch + e3];

% Auxiliary constraints for terminal cost
for i = 1:1:length(b_cf)
	constr = [constr;
		a_cf(:,i)'*x{N+1}([11,12]) + b_cf(i) <= q];
end

constr = [constr;
	e1 >= 0;
	e2 >= 0];

% ===== Optimizer object =====
params_in = x1_in;
sol_out = u{1};

options = sdpsettings('verbose',0,'solver','quadprog');
lmpc = optimizer(constr,cost,options,params_in,sol_out);

u1 = lmpc(x1);

end















