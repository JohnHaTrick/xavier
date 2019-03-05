function [ e_opt, Uy_opt, Fy_opt, cost ] = calc_Fy_MPC(x0, e_des, e_bounds, Fy_max, m, dt, N_PH)

x0 = [x0(2); x0(4)];            % x0 MPC = [e; Uy]

% lateral dynamics
Ac     = [0 1; 0 0];            % e_dot  = Uy
Bc     = [  0; 1000/m];         % Uy_dot = Fy / m
[A, B] = myc2d(Ac, Bc, dt);     % discretize dynamics
N_x    = size(A,1);             % state vector size: x0:2
N_u    = size(B,2);             % input vector size: Fy-1

% cost weights
Q_e     = 10;
Q_Uy    = 0;
% Q_end = ?                     % terminal cost?
R_Fy    = 1;
% lamda   = 1000;                 % slack cost
Q_stage = [Q_e 0; 0 Q_Uy];      % stage-wise cost matrices
Q_cell  = repmat({Q_stage}, 1, N_PH);
Q       = blkdiag(Q_cell{:});   % horizon-spanning cost matrices
R_stage = R_Fy * eye(N_u);
R_cell  = repmat({R_stage}, 1, N_PH-1);
R       = blkdiag(R_cell{:});

e_idxs        = 1 : N_x : 2*N_PH-1; % x indices which point to e states
e_min         = e_bounds(:,1);
e_max         = e_bounds(:,2);
x_des         = zeros(N_x*N_PH,1);
x_des(e_idxs) = e_des;
buffer        = 0.5;

%% solve convex problem 
cvx_begin quiet
    variable x(N_x*N_PH, 1)
    variable u(N_u*(N_PH-1), 1)
    variable slack
    
    minimize((x-x_des)'*Q*(x-x_des) + u'*R*u); % quadratic form 
    
    subject to
        for i = 1 : (N_PH-1)
           x( N_x*i+1 : N_x*(i+1) ) == ...
               A * x( N_x*(i-1)+1 : N_x*i ) + B * u(N_u*i);
        end
        x(1:N_x)  == x0;           % init condition
        u         >= -1*Fy_max;    % Steering constraints
        u         <=    Fy_max;
        x(e_idxs) <= e_max-buffer; % don't crash left
        x(e_idxs) >= e_min+buffer; %         or right
%         slack     >= 0;            % slack must be positive
cvx_end

%% parse results
Fy_opt   = u;                       % input
x_opt    = reshape(x, [N_x, N_PH]); % states
e_opt    = x_opt(1,:);
Uy_opt   = x_opt(2,:);
cost.sum = cvx_optval;
e_des_v  = e_des*ones(size(e_opt));
cost.e   = sum((e_opt-e_des_v) .* (e_opt-e_des_v)) * Q_e;  % cost
% cost.Uy  = sum(Uy_opt .* Uy_opt) * Q_Uy;
cost.Fy  = sum(Fy_opt .* Fy_opt) * R_Fy;

end