function [x_safe, u_safe, Q_safe] = LMPC(x1, x_first, u_first, N, n_iter, data, rep, anim_plot)

x1_init = x1;

% Store first trajectory

x_safe{1} = x_first;
u_safe{1} = u_first;
Q_safe{1} = compute_cost(x_first,u_first,data);

% Initialize SS

SS = x_safe{1}([11,12],:);
Q_SS = Q_safe{1};

% SS and Q_SS relaxation

[A_ch, b_ch] = points_to_ch_lcon(SS);

k_cf = 3;
[a_cf, b_cf, ~] = convex_piecewise_fit(SS,Q_SS,k_cf,k_cf*5);
while length(b_cf) < 3
	k_cf = k_cf + 1;
	[a_cf, b_cf, ~] = convex_piecewise_fit(SS,Q_SS,k_cf,k_cf*5);
end

% x = [z  phi  theta  psi  vx  vy  vz  v_phi  v_theta  v_psi  s  d  t]
% u = [u1  u2  u3  u4]

for j = 1:1:n_iter
	
	x_lmpc = x1;
	u_lmpc = [];

	u1 = [0; 0; 0; 0];
	
	% ===== BEGIN Animated plot (1) =====
	figure(1)
	
	% Track
	subplot(1,2,1)
	
	[q_loc_x, q_loc_y] = plot_on_track(data.track,x1(11),x1(12));
	q_loc = plot(q_loc_x,q_loc_y,'r.','markersize',15);
	
	% Altitude
	subplot(1,2,2)
	
	q_alt = plot(x1(11),x1(1),'r.','markersize',15);
	
	step_disp = 1;
	% ===== END Animated plot (1) =====
	
	while x1(11) < data.L_track
		
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
		
		% ===== BEGIN LMPC =====
		% Get optimal control input from LMPC algorithm
		u1 = LMPC_OP(x1, A, B, c, A_ch, b_ch, a_cf, b_cf, N, data, rep);

		% Apply optimal control input and get next states
		x2 = quadrot_nl(x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),...
			x1(7),x1(8),x1(9),x1(10),x1(11),x1(12),x1(13),...
			u1(1),u1(2),u1(3),u1(4),data.K_fun_real(x1(11)),data.T);

		% Store optimal states and inputs
		x_lmpc = [x_lmpc, x2];
		u_lmpc = [u_lmpc, u1];
		% ===== END LMPC =====
		
		% ===== BEGIN Animated plot (2) =====
		subplot(1,2,1)
	
		delete(q_loc);

		[x_traj_1, y_traj_1] = plot_on_track(data.track,x1(11),x1(12));
		[x_traj_2, y_traj_2] = plot_on_track(data.track,x2(11),x2(12));
		plot([x_traj_1, x_traj_2],[y_traj_1, y_traj_2],'b-','linewidth',1)

		[q_loc_x, q_loc_y] = plot_on_track(data.track,x2(11),x2(12));
		q_loc = plot(q_loc_x,q_loc_y,'r.','markersize',15);

		if anim_plot == true
			drawnow
		end

		subplot(1,2,2)

		delete(q_alt);

		plot([x1(11), x2(11)], [x1(1), x2(1)], 'b-', 'linewidth', 1)

		q_alt = plot(x2(11),x2(1),'r.','markersize',15);

		if anim_plot == true
			drawnow
		end
		
		clc
		fprintf('Iteration %d\n', j);
		fprintf('Time = %1.2f s\n',step_disp*data.T);
		
		fprintf('Previous iteration (%d):\nLap time = %1.2f s\nCost = %1.10e\n', ...
			j-1, length(x_safe{j,1})*data.T, Q_safe{j,1}(1))
		
		step_disp = step_disp + 1;
		% ===== END Animated plot (2) =====

		% Update initial states
		x1 = x2;
		
	end
	
	x_safe{end+1,1} = x_lmpc;
	u_safe{end+1,1} = u_lmpc;
	Q_safe{end+1,1} = compute_cost(x_lmpc,u_lmpc,data);
	
	% Update SS
	SS = [SS, x_safe{end,1}([11,12],:)];
	Q_SS = [Q_SS, Q_safe{end,1}];

	% SS relaxation
	[A_ch, b_ch] = points_to_ch_lcon(SS);
	
	% Update initial state for next iteration
	
	if rep == true
		x1 = x_lmpc(:,end);
		x1(11) = x1(11) - data.L_track;
		x1(13) = x1(13) - 2*pi;
	else
		x1 = x1_init;
	end
	
	delete(q_loc);
	delete(q_alt);
	
end

clc

fprintf('Iteration cost:\n');
for j=1:1:n_iter+1
	fprintf('%d\t%1.10e\n', j-1, Q_safe{j,1}(1));
end

fprintf('Lap time:\n');
for j=1:1:n_iter+1
	fprintf('%d\t%1.2f s\n', j-1, length(x_safe{j,1})*data.T);
end

end


















