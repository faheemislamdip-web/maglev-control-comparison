clc;
close all;
clear;

% SYSTEM PARAMETERS
A = [0 1;
     19600 0];
B = [0; -19.8];
C = [1 0];
D = 0;
sys = ss(A, B, C, D);

% FIXED-POINT LQR CONTROLLER
Q = diag([100 1]);   % Penalize position heavily, velocity lightly
R = 1;                % Penalize control effort

K_LQR = lqr(A, B, Q, R);
Acl = A - B*K_LQR;

disp('LQR Gain K = ');
disp(K_LQR);
disp('Closed-loop Eigenvalues = ');
disp(eig(Acl));

% SIMULATION
dt = 0.0001;
T = 2;
t = 0:dt:T;

x0 = [0.01; 0];   % Initial state: [position deviation; velocity deviation]

x_LQR = zeros(2, length(t));
x_LQR(:,1) = x0;
u_LQR = zeros(1, length(t));

for i = 1:length(t)-1
    u_LQR(i) = -K_LQR * x_LQR(:,i);
    dx = A*x_LQR(:,i) + B*u_LQR(i);
    x_LQR(:,i+1) = x_LQR(:,i) + dx*dt;   % Euler integration
end

% PLOTTING
figure;
subplot(2,1,1);
plot(t, x_LQR(1,:), 'LineWidth', 2);
grid on;
title('Fixed-Point LQR Position Response');
xlabel('Time (s)');
ylabel('Position Deviation (m)');

subplot(2,1,2);
plot(t, u_LQR, 'LineWidth', 2);
grid on;
title('Fixed-Point LQR Control Input');
xlabel('Time (s)');
ylabel('Control Input (Delta i)');