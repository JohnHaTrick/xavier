%% Linear Lateral MPC Playground
% Author: John Alsterda
%         adapted from Nitin Kapania
% point-mass with fixed long. speed navigates a potential obstacle

clear all;
% close all;
clc;

%% Parameters
% physical
m      = 1000;                      % [kg] vehicle mass
mu     = 1;                         % friction coefficient
g      = 9.81;                      % [m/s2] gravity
Fy_max = mu*m*g/1000;               % [kN] max lateral force

% discretization
dt    = 0.05;                       % [sec] time step
T     = 4;                          % [sec] simulate until T
t     = 0:dt:2*T-dt;                % time array
N     = length(t)/2;                % num steps in simulation
N_PH  = 30;                         % num steps in prediction horizon

% world
e_des = -1.0;                       % [m] where should car go?
e_min = -2.0;                       % [m] road boundaries (use for obs)
e_max =  2.0;                       %
obs.s_min = 15;
obs.s_max = 18;
obs.e_min = -0.5;
obs.e_max =  2.0;

% state arrays
Ux  = 10;                           % [m/s]  long. velocity (const)
s   = linspace(0,2*Ux(1)*T,2*N+1);  % [m]    position (known apriori)
Fy  = nan(N,1);                     % [kN]   lat. force
dUy = nan(N,1);                     % [m/s2] lat. acceleration
Uy  = nan(N,1); Uy(1) = 0.0;        % [m/s]  lat. velocity
e   = nan(N,1); e(1)  = e_des;      % [m]    lat. deviation

%% preallocate MPC arrays
% optimal solutions
opt  = struct('t', cell(N,1), 's', cell(N,1), ...
             'e', cell(N,1), 'Uy',cell(N,1), 'Fy',cell(N,1));
% opt.slack = nan(1,N); % slack var. (one per iteration?)

% optimal costs
cost = struct('sum',cell(N,1), 'e', cell(N,1), 'Fy',cell(N,1));
% cost.slack = nan(N_PH,N);

e_bounds  = nan(N_PH,   2);          % dynamic e bounds  

for i = 1:N-1
    disp(['iteration ',num2str(i),' of ',num2str(N)])
    
    x0 = [s(i), e(i), Ux, Uy(i)];    % Set current state &
    opt(i).t = t(i:i+N_PH-1);        %   PH time &
    opt(i).s = s(i:i+N_PH-1);        %   long. progress (known apriori)
    
    for j = 1:N_PH                   % adjust e bounds for obstacle
        if s(i+j-1) >= obs.s_min && s(i+j-1) <= obs.s_max
            e_bounds(j,:) = [obs.e_min; obs.e_max];
        else
            e_bounds(j,:) = [e_min; e_max];
        end
    end
                                     % call MPC
    [opt(i).e, opt(i).Uy, opt(i).Fy, cost(i)] = ...
        calc_Fy_MPC(x0, e_des, e_bounds, Fy_max, m, dt, N_PH);
    
    Fy(i)   = opt(i).Fy(1);          % parse output
    dUy(i)  = Fy(i)/m;
    Uy(i+1) = opt(i).Uy(2);
    e(i+1)  = opt(i).e(2);
end

%% plot things
plot_MPC(t,s(1:N),e,Fy,Uy,e_min,e_max,obs,opt,cost);
