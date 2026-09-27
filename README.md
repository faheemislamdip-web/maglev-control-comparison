# maglev-control-comparison
 "Comparative Evaluation of Control Strategies for a Magnetic Levitation System: LQR, Feedback Linearization, and PID"
# Comparative Evaluation of Control Strategies for a Magnetic Levitation System

## Motivation
This project is part of Modelling & Simulation course. I had to the chance to do 
an open-ended project, I used the opportunity to do a project on trains, since I like 
trains a lot both in terms of engineering and travelling.
Magnetic levitation is a classic benchmark in control theory: the ball 
is held up purely by an electromagnet's pull, but this equilibrium is 
inherently unstable — no fixed current can sustain it. I wanted to 
explore why and to what range current control strategies work and to 
verify this rigorously rather than just assume 
it — using nonlinear simulation, Monte Carlo testing, and optimization, 
not just eigenvalue math.

## What this project does
1. Proves the system is open-loop unstable (both analytically and via simulation)
2. Designs and compares four controllers: PID, Fixed-Point LQR, 
   Gain-Scheduled LQR, and Feedback Linearization
3. Tests each controller's robustness under sensor noise and disturbance 
   (Monte Carlo, 100 trials)
4. Uses Particle Swarm Optimization to independently verify whether 
   better gains exist

Fixed-point LQR and PID both stabilize the system for small disturbances 
(~0.1mm) but diverge completely for a 5mm disturbance — a scenario 
Feedback Linearization handles cleanly, since it cancels the plant's 
nonlinearity exactly rather than approximating it near one point.

## Monte Carlo Robustness Comparison

| Controller | Settling (s) | Steady-State Error (m) | Failure Rate |
|---|---|---|---|
| PID | 0.9999 | 4.90 | ~100% |
| Fixed LQR | 0.9999 | 4.91 | ~100% |
| Feedback Linearization | 0.050 | 0.00001 | 0% |



