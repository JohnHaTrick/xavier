%% Toy MPC Playground
% Author: John Alsterda
%         adapted from Nitin Kapania
% single-dim plant with trivial dynamics

clear all;
close all;
clc;

%% Parameters
x0    = 0;                                      % initial state
t0    = 0;                                      %   and time
x_min = 1;                                      % min x to escape obstacle
t_obs = 6;                                      % obstacle pops at this time
% u_max = 1;                                      % max input command
dt   = 1;                                       % discretization time step
T    = 9;                                       % simulate until T
N_t  = T/dt;                                    % num time steps in sim
N_PH = 6;                                       % num steps in pred. horizon
P    = [0.5 0.25 0.0];                          % obstacle probabilities

%% preallocate MPC arrays
opt_r   = struct('t',   cell(N_t,1), ...         % robust
                 'x',   cell(N_t,1), ...         %  optimal trajectory
                 'u',   cell(N_t,1));
cost_r  = struct('sum', cell(N_t,1), ...         %   and costs
                 'u',   cell(N_t,1));
opt_50  = struct('t',   cell(N_t,1), ...         % 50% contingency
                 'x_n', cell(N_t,1), ...         %   optimal trajectory
                 'u_n', cell(N_t,1), ...
                 'x_c', cell(N_t,1), ...
                 'u_c', cell(N_t,1));
cost_50 = struct('sum', cell(N_t,1), ...         %   and costs
                 'u_n', cell(N_t,1), ...
                 'u_c', cell(N_t,1));
opt_25  = struct('t',   cell(N_t,1), ...         % 25% contingency
                 'x_n', cell(N_t,1), ...         %   optimal trajectory
                 'u_n', cell(N_t,1), ...
                 'x_c', cell(N_t,1), ...
                 'u_c', cell(N_t,1));
cost_25 = struct('sum', cell(N_t,1), ...         %   and costs
                 'u_n', cell(N_t,1), ...
                 'u_c', cell(N_t,1));
opt_c   = struct('t',   cell(N_t,1), ...         % 0% contingency
                 'x_n', cell(N_t,1), ...         %   optimal trajectory
                 'u_n', cell(N_t,1), ...
                 'x_c', cell(N_t,1), ...
                 'u_c', cell(N_t,1));
cost_c  = struct('sum', cell(N_t,1), ...         %   and costs
                 'u_n', cell(N_t,1), ...
                 'u_c', cell(N_t,1));

%% MPC and forward sim
x0_r = x0;  x0_50 = x0; x0_25 = x0; x0_c = x0;
for i = 1:N_t
    disp(['iteration ',num2str(i),' of ',num2str(N_t)])
    
    k_obs  = t_obs - t0 + 1;                    % Set obs stage in MPC
    
    opt_r(i).t  = t0 : dt : t0+dt*(N_PH-1);     % Set PH times
    opt_50(i).t = opt_r(i).t;
    opt_25(i).t = opt_r(i).t;
    opt_c(i).t  = opt_r(i).t;
    
    [opt_r(i).x, opt_r(i).u, cost_r(i)] = ...   % call RMPC
        calc_RMPC(x0_r, x_min, k_obs, N_PH);
    
    [opt_50(i).x_n, opt_50(i).u_n, ...          % call 50% CMPC
        opt_50(i).x_c, opt_50(i).u_c, ...
        cost_50(i)] = ...
        calc_CMPC(x0_50, x_min, k_obs, N_PH, P(1));
    
    [opt_25(i).x_n, opt_25(i).u_n, ...          % call 25% CMPC
        opt_25(i).x_c, opt_25(i).u_c, ...
        cost_25(i)] = ...
        calc_CMPC(x0_25, x_min, k_obs, N_PH, P(2));
    
    [opt_c(i).x_n, opt_c(i).u_n, ...            % call CMPC
        opt_c(i).x_c, opt_c(i).u_c, ...
        cost_c(i)] = ...
        calc_CMPC(x0_c, x_min, k_obs, N_PH, P(3));
    

    t0    = t0 + dt;
    x0_r  = opt_r(i).x(2);                       % set next robust x0
    x0_50 = opt_50(i).x_n(2);                    %   & 50% contingency x0
    x0_25 = opt_25(i).x_n(2);                    %   & 25% contingency x0
    x0_c  = opt_c(i).x_n(2);                     %   &  0% contingency x0
end

%% plot things
plot_MPC(opt_r,cost_r,opt_50,cost_50,opt_25,cost_25,opt_c,cost_c,t_obs,x_min);
