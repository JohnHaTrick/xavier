function [ x_n, u_n, x_c, u_c, cost ] = ...
    calc_CMPC(x0, x_min, k_obs, N_PH)

% cost weights
R_n = 1;                                        % input weight
R_c = 1/1000;
% lamda = 1000;                                 % slack cost

%% solve convex problem 
cvx_begin
    variable x_n(N_PH,   1)                     % nominal vars
    variable u_n(N_PH-1, 1)
    variable x_c(N_PH,   1)                     % contingency vars
    variable u_c(N_PH-1, 1)
%     variable slack
    
    minimize( u_n'*R_n*u_n + u_c'*R_c*u_c );    % quadratic cost 
    
    subject to
        for k = 1 : (N_PH-1)
           x_n(k+1) == x_n(k) + u_n(k);
           x_c(k+1) == x_c(k) + u_c(k);
        end
        x_n(1) == x0                            % init condition
        x_c(1) == x0
        if k_obs >= 1 && k_obs <= N_PH
            x_c(k_obs) >= x_min;                % don't crash!
        end
        u_n(1) == u_c(1)                        % coupling constraint
%         slack     >= 0;                         % slack must be positive
cvx_end

%% parse results
cost.sum = cvx_optval;
cost.u_n = sum(u_n .* u_n) * R_n;
cost.u_c = sum(u_c .* u_c) * R_c;

end