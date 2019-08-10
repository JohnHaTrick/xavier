%% Toy MPC Playground
% Author: John Alsterda
%         adapted from Nitin Kapania
% single-dim plant with trivial dynamics

clear all;
close all;
clc;

%% Parameters
x0    = 0;                                      % initial state
y0    = 0;                                      %   x, y, & time
t0    = 0;
v_y   = 1;                                      % speed in y dir. (const)(y=t)
dt    = 1;                                      % time step discretization
T     = 13;                                     % total Time in sim
N_t   = T/dt;                                   % num time steps in sim
x_min = 1; buff  = 0.1;                         % min x to escape obstacle, and visual buffer
y_obs = 10; v_obs = (x_min+1)/6;                % obs may pop at this y and speed
N_PH  = 10;                                     % num steps in pred. horizon
P     = [0.5 0.25 0.0];                         % obstacle probabilities

%% preallocate MPC arrays
opt_A   = struct('t',     cell(N_t,1), ...      % common values: t
                 'y',     cell(N_t,1), ...      %                y
                 'x_min', cell(N_t,1));         %                x_min
opt_r   = struct('x',     cell(N_t,1), ...      % robust
                 'u',     cell(N_t,1));         %  optimal trajectory
cost_r  = struct('sum',   cell(N_t,1), ...      %   and costs
                 'u',     cell(N_t,1), ...
                 'cum',   cell(N_t,1));
opt_50  = struct('x_n',   cell(N_t,1), ...      % 50% contingency
                 'u_n',   cell(N_t,1), ...      %   optimal trajectory
                 'x_c',   cell(N_t,1), ...
                 'u_c',   cell(N_t,1));
cost_50 = struct('sum',   cell(N_t,1), ...      %   and costs
                 'u_n',   cell(N_t,1), ...
                 'u_c',   cell(N_t,1), ...
                 'cum',   cell(N_t,1));
opt_25  = struct('x_n',   cell(N_t,1), ...      % 25% contingency
                 'u_n',   cell(N_t,1), ...      %   optimal trajectory
                 'x_c',   cell(N_t,1), ...
                 'u_c',   cell(N_t,1));
cost_25 = struct('sum',   cell(N_t,1), ...      %   and costs
                 'u_n',   cell(N_t,1), ...
                 'u_c',   cell(N_t,1), ...
                 'cum',   cell(N_t,1));
opt_c   = struct('x_n',   cell(N_t,1), ...      % 0% contingency
                 'u_n',   cell(N_t,1), ...      %   optimal trajectory
                 'x_c',   cell(N_t,1), ...
                 'u_c',   cell(N_t,1));
cost_c  = struct('sum',   cell(N_t,1), ...      %   and costs
                 'u_n',   cell(N_t,1), ...
                 'u_c',   cell(N_t,1), ...
                 'cum',   cell(N_t,1));

%% MPC and forward sim
x0_r = x0;  x0_50 = x0; x0_25 = x0; x0_c = x0;
for i = 1:N_t
    disp(['iteration ',num2str(i),' of ',num2str(N_t)])
    
    k_obs   = y_obs - y0 + 1;                   % Set obs stage in MPC
    x_min_k = min(v_obs*max(y_obs-y0,0)-1, ...  % how far can obs reach
                  x_min)     + buff;            %   by time we get there?
    opt_A(i).x_min = x_min_k - buff;

    opt_A(i).t  = t0 : dt  : t0 + dt *(N_PH-1); % Set PH times
    opt_A(i).y  = y0 : v_y : y0 + v_y*(N_PH-1); %   and y positions
    
    [opt_r(i).x, opt_r(i).u, cost_r(i)] = ...   % call RMPC
        calc_RMPC(x0_r, x_min_k, k_obs, N_PH);
    
    [opt_50(i).x_n, opt_50(i).u_n, ...          % call 50% CMPC
        opt_50(i).x_c, opt_50(i).u_c, ...
        cost_50(i)] = ...
        calc_CMPC(x0_50, x_min_k, k_obs, N_PH, P(1));
    
    [opt_25(i).x_n, opt_25(i).u_n, ...          % call 25% CMPC
        opt_25(i).x_c, opt_25(i).u_c, ...
        cost_25(i)] = ...
        calc_CMPC(x0_25, x_min_k, k_obs, N_PH, P(2));
    
    [opt_c(i).x_n, opt_c(i).u_n, ...            % call CMPC
        opt_c(i).x_c, opt_c(i).u_c, ...
        cost_c(i)] = ...
        calc_CMPC(x0_c, x_min_k, k_obs, N_PH, P(3));
    
    if i > 1                                    % fix cumulative
        cost_r( i).cum = cost_r( i).cum + cost_r( i-1).cum;
        cost_25(i).cum = cost_25(i).cum + cost_25(i-1).cum;
        cost_50(i).cum = cost_50(i).cum + cost_50(i-1).cum;
        cost_c( i).cum = cost_c( i).cum + cost_c( i-1).cum;
    end
    
    y0    = y0 + v_y;                           % set y0 and t0
    t0    = t0 + dt;                            %   for next MPC PHs
    x0_r  = opt_r(i).x(2);                      % set next robust x0
    x0_50 = opt_50(i).x_n(2);                   %   & 50% contingency x0
    x0_25 = opt_25(i).x_n(2);                   %   & 25% contingency x0
    x0_c  = opt_c(i).x_n(2);                    %   &  0% contingency x0
end

%% plot things
plot_MPC(opt_A,opt_r,cost_r,opt_50,cost_50,opt_25,cost_25,opt_c,cost_c,y_obs);
