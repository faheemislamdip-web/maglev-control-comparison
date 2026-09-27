clc; close all; clear;

% SYSTEM PARAMETERS (standard set)
m = 0.1;
g = 9.81;
k = 0.0001;
x0 = 0.01;
i0 = x0*sqrt(m*g/k);

% VIRTUAL-INPUT PD GAINS 
wn = 100;      % desired natural frequency 
zeta = 1;      % desired damping ratio 

Kp = wn^2;
Kd = 2*zeta*wn;

fprintf('Feedback linearization PD gains: Kp=%.2f, Kd=%.2f\n', Kp, Kd);

% SIMULATION (TRUE NONLINEAR PLANT)
dt = 0.0001;
T = 1.0;
t = 0:dt:T;
N = length(t);

x = zeros(1,N);
v_ = zeros(1,N);
i_cmd = zeros(1,N);

x(1) = x0 + 0.005;   % a LARGE deviation (5mm) test point:
v_(1) = 0;          

for n = 1:N-1
    pos = x(n);
    vel = v_(n);

    e = pos - x0;
    v = -Kp*e - Kd*vel;

    v = min(v, g - 0.5);   % SATURATE v so (g - v) stays positive 

    i_cmd(n) = pos * sqrt(m*(g - v)/k);   % exact nonlinearity-canceling current

    F = k * i_cmd(n)^2 / pos^2;           % TRUE nonlinear force 
    accel = (m*g - F) / m;

    v_(n+1) = vel + accel*dt;
    x(n+1) = pos + v_(n+1)*dt;

    if isnan(x(n+1)) || abs(x(n+1)) > 10
        fprintf('DIVERGED at t = %.4f s\n', t(n));
        break;
    end
end

% PLOTTING
figure;
subplot(2,1,1);
plot(t, x, 'LineWidth', 2);
yline(x0, '--r');
grid on;
title('Feedback Linearization: Position Response (5mm initial deviation)');
xlabel('Time (s)'); ylabel('Position (m)');

subplot(2,1,2);
plot(t, i_cmd, 'LineWidth', 2);
grid on;
title('Commanded Current');
xlabel('Time (s)'); ylabel('Current (A)');