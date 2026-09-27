clc; close all; clear;

% SYSTEM PARAMETERS (standard set)
m = 0.1;
g = 9.81;
k = 0.0001;
x0 = 0.01;
i0 = x0*sqrt(m*g/k);

A = [0 1; 19600 0];
B = [0; -19.8];
C = [1 0];

% DERIVE PID GAINS VIA AUGMENTED LQR
% Augment with an integral-of-error state: z_dot = e = x1 - r (r=0 -> e=x1)
A_aug = [A, zeros(2,1); -C, 0];
B_aug = [B; 0];

Q_aug = diag([1000, 1, 100]);   % [position, velocity, integral] weighting
R_aug = 0.1;

K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

Kp = K_aug(1);
Kd = K_aug(2);
Ki = K_aug(3);

fprintf('=== PID gains from augmented LQR ===\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);

Acl_aug = A_aug - B_aug*K_aug;
fprintf('\nClosed-loop eigenvalues (with integral state):\n');
disp(eig(Acl_aug));

% SIMULATION ON TRUE NONLINEAR PLANT (with current saturation)
dt = 0.0001;
T = 1.0;
t = 0:dt:T;
N = length(t);

x = zeros(1,N);
vel = zeros(1,N);
u = zeros(1,N);
ie = 0;

x(1) = x0 + 0.0001;   % start with a small disturbance
vel(1) = 0;

I_MAX = 5;   % realistic current saturation limit (A) -- current cannot be negative
I_MIN = 0;

for n = 1:N-1
    pos = x(n);
    v = vel(n);

    error = pos - x0;   % convention: measured - target
    ie = ie + error*dt;

    u(n) = -(Kp*error + Kd*v + Ki*ie);   % control law: u = -K_aug * [pos; vel; integral]

    i_total = i0 + u(n);
    i_total = max(min(i_total, I_MAX), I_MIN);   % SATURATE current -- prevents
                                                   % the i^2 sign-invariance trap
    F = k * i_total^2 / pos^2;
    accel = (m*g - F) / m;

    vel(n+1) = v + accel*dt;
    x(n+1) = pos + vel(n+1)*dt;

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
title('PID Position Response, true nonlinear plant');
xlabel('Time (s)'); ylabel('Position (m)');

subplot(2,1,2);
plot(t, u, 'LineWidth', 2);
grid on;
title('PID Control Input (Delta i)');
xlabel('Time (s)'); ylabel('Delta i');