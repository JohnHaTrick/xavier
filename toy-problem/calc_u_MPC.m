function [ x_opt, u_opt, cost ] = calc_u_MPC(x0, x_min, k_obs, N_PH)

% cost weights
R = 1;                          % input
% lamda = 1000;                 % slack cost

%% solve convex problem 
cvx_begin
    variable x(N_PH, 1)         % states
    variable u(N_PH, 1)         % inputs
%     variable slack
    
    minimize( u'*R*u );         % quadratic cost 
    
    subject to
        for k = 1 : (N_PH-1)
           x(k+1) == x(k) + u(k+1);
        end
        x(1) == x0;             % init condition
        if k_obs >= 1 && k_obs <= N_PH
            x(k_obs) >= x_min;  % don't crash!
        end
%         slack     >= 0;       % slack must be positive
cvx_end

%% parse results
x_opt    = x; %reshape(x, [1, N_PH]);
u_opt    = u;
cost.sum = cvx_optval;
cost.u_n = sum(u .* u) * R;

end