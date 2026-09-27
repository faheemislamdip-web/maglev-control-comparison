clc; close all; clear;

% PLANT DEFINITION 
plant.m  = 0.1;
plant.k  = 0.0001;
plant.x0 = 0.01;
plant.g  = 9.81;
plant.i0 = plant.x0*sqrt(plant.m*plant.g/plant.k);

% CONTROLLER GAINS TO TEST 
controllers = struct('name',{},'type',{},'params',{});
controllers(1) = struct('name','PID',                'type','PID',    'params',[1985.2972, 31.6228, 14.5098]);
controllers(2) = struct('name','Fixed LQR',           'type','LQR',    'params',[1979.8, 14.2]);
controllers(3) = struct('name','Gain-Scheduled LQR',  'type','GS_LQR', 'params',[1979.8,14.2, 1979.9,14.3, 1980.1,14.5, 0.002,0.01]);
controllers(4) = struct('name','Feedback Linearization','type','FL',   'params',[10000, 200]);

% MONTE CARLO SETTINGS
numTrials    = 100;
noise_std    = 1e-5;
dist_time    = 0.3;
dist_mag_max = 0.02;

% MONTE CARLO FOR EACH CONTROLLER
fprintf('%-25s %10s %10s %12s %10s %10s %8s\n', ...
    'Controller','Settle(s)','Std','Overshoot(m)','SSErr(m)','Effort','Fail%');

for c = 1:length(controllers)
    ctrl = controllers(c);

    settling  = zeros(1,numTrials);
    overshoot = zeros(1,numTrials);
    ss_err    = zeros(1,numTrials);
    effort    = zeros(1,numTrials);
    fail_count = 0;

    for trial = 1:numTrials
        dist_mag = (rand()-0.5)*2*dist_mag_max;
        m = simulate_all(ctrl.type, ctrl.params, plant, noise_std, dist_time, dist_mag);

        settling(trial)  = m.settling_time;
        overshoot(trial) = m.overshoot;
        ss_err(trial)    = m.ss_error;
        effort(trial)    = m.control_effort;
        if m.failed
            fail_count = fail_count + 1;
        end
    end

    fprintf('%-25s %10.4f %10.4f %12.6f %10.6f %10.4f %8.1f\n', ...
        ctrl.name, mean(settling), std(settling), mean(overshoot), ...
        mean(ss_err), mean(effort), fail_count/numTrials*100);
end