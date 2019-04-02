%% Toy MPC Playground
% Author: John Alsterda
%         adapted from Nitin Kapania
% single-dim plant with trivial dynamics

clear all;
close all;
clc;

%% Parameters                               % physical
x0    = 0;
x_min = 1;                                  % min x to escape obstacle
t_obs = 5;                                  % obstacle pops at this time
% u_max = 10;                               % max input command

                                            % discretization
dt   = 1;                                   % time step
T    = 7;                                   % simulate until T
N_t  = T/dt;                                % num time steps in sim
t    = 0:dt:2*T-dt;                         % time array (*2 to hold PH)
N_PH = 5;                                   % num steps in pred. horizon

%% preallocate MPC arrays
opt  = struct('t',   cell(N_t,1), ...       % optimal solutions
              'x',   cell(N_t,1), ...
              'u',   cell(N_t,1));          % slack?

cost = struct('sum', cell(N_t,1), ...       % optimal costs
              'u_n', cell(N_t,1));          % slack or x or u_c?

%% MPC and forward sim
for i = 1:N_t % need minus one?
    disp(['iteration ',num2str(i),' of ',num2str(N_t)])
    
    opt(i).t = t(i:i+N_PH-1);               % Set PH time &
    k_obs    = t_obs - opt(i).t(1) + 1;     %   obs stage in MPC
    
    [opt(i).x, opt(i).u, cost(i)] = ...     % call MPC
        calc_u_MPC(x0, x_min, k_obs, N_PH); % add u_max here?

    x0 = opt(i).x(2);            % set next x0
end

%% plot things
plot_MPC(opt,cost,t_obs,x_min);
