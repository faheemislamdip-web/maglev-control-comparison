clc;
close all;
clear;

% SYSTEM PARAMETERS
m = 0.1;        % ball mass (kg)
g = 9.81;       % gravity (m/s^2)
k = 0.0001;     % magnetic force constant (N*m^2/A^2)
x0 = 0.01;      % chosen equilibrium gap (m)

% SYMBOLIC LINEARIZATION (TAYLOR SERIES)
syms xs is

F = k*is^2/xs^2;          % nonlinear magnetic force

i0 = x0*sqrt(m*g/k);      % equilibrium current, solved from F(x0,i0) = m*g

% Partial derivatives (first-order Taylor expansion terms)
dF_dx = diff(F, xs);
dF_di = diff(F, is);

% Evaluate partial derivatives At equilibrium
dFdx_val = double(subs(dF_dx, {xs, is}, {x0, i0}));
dFdi_val = double(subs(dF_di, {xs, is}, {x0, i0}));

% BUILD STATE-SPACE MATRICES
A = [0, 1;
    -dFdx_val/m, 0];
B = [0; -dFdi_val/m];
C = [1 0];
D = 0;

disp('Equilibrium current i0 = ');
disp(i0);
disp('A = ');
disp(A);
disp('B = ');
disp(B);

% STABILITY CHECK (OPEN-LOOP)
disp('Open-loop eigenvalues (eig(A)) = ');
disp(eig(A));
% A positive eigenvalue confirms open-loop instability.

