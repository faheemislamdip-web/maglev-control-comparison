clc; close all; clear;

% PLANT DEFINITION 
plant.m  = 0.1;
plant.k  = 0.0001;
plant.x0 = 0.01;
plant.g  = 9.81;
plant.i0 = plant.x0*sqrt(plant.m*plant.g/plant.k);

% COST FUNCTION WEIGHTS
w1 = 1000;   % settling time
w2 = 500;    % overshoot
w3 = 0.001;  % control effort
w4 = 2000;   % steady-state error
penalty_fail = 1e6;

options = optimoptions('particleswarm', 'SwarmSize', 40, 'MaxIterations', 60, ...
                        'Display', 'iter');

% PSO: PID  [Kp, Ki, Kd]
cost_PID = @(p) local_cost('PID', p, plant, w1,w2,w3,w4,penalty_fail);
lb = [0, 0, 0];  
ub = [3000, 100, 100];

[best_PID, J_PID] = particleswarm(cost_PID, 3, lb, ub, options);
fprintf('Best PID: Kp=%.4f, Ki=%.4f, Kd=%.4f  (cost=%.4f)\n', best_PID(1), best_PID(2), best_PID(3), J_PID);

% PSO: Fixed LQR  [K1, K2]
cost_LQR = @(p) local_cost('LQR', p, plant, w1,w2,w3,w4,penalty_fail);
lb = [0, 0]; ub = [3000, 100];

[best_LQR, J_LQR] = particleswarm(cost_LQR, 2, lb, ub, options);
fprintf('Best LQR: K1=%.4f, K2=%.4f  (cost=%.4f)\n', best_LQR(1), best_LQR(2), J_LQR);

% PSO: Feedback Linearization  [Kp_fl, Kd_fl]
cost_FL = @(p) local_cost('FL', p, plant, w1,w2,w3,w4,penalty_fail);
lb = [0, 0]; ub = [50000, 1000];

[best_FL, J_FL] = particleswarm(cost_FL, 2, lb, ub, options);
fprintf('Best FL: Kp=%.4f, Kd=%.4f  (cost=%.4f)\n', best_FL(1), best_FL(2), J_FL);

% LOCAL COST FUNCTION
function J = local_cost(ctrl_type, params, plant, w1,w2,w3,w4,penalty_fail)
    m = simulate_all(ctrl_type, params, plant, 0, Inf, 0);
    if m.failed
        J = penalty_fail;
    else
        J = w1*m.settling_time + w2*m.overshoot + w3*m.control_effort + w4*m.ss_error;
    end
end