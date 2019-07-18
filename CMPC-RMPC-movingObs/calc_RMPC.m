function [ x, u, cost ] = ...
    calc_RMPC(x0, x_min, k_obs, N_PH)

% cost weights
R = 1;                          % input
% lamda = 1000;                 % slack cost

%% solve convex problem 
cvx_begin
    variable x(N_PH,   1)       % states
    variable u(N_PH-1, 1)       % inputs
%     variable slack
    
    minimize( u'*R*u );         % quadratic cost 
    
    subject to
        for k = 1 : (N_PH-1)
           x(k+1) == x(k) + u(k);
        end
        x(1) == x0;             % init condition
        if k_obs >= 1 && k_obs <= N_PH
            x(k_obs) >= x_min;  % don't crash!
        end
%         slack     >= 0;       % slack must be positive
cvx_end

%% parse results
cost.sum = cvx_optval;
cost.u   = sum(u .* u) * R;
cost.cum = u(1)^2      * R;     % NOT FINSIHED! Needs add previous cum in main

end