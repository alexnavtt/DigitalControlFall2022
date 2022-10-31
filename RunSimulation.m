clc
clear
close all

%% Define constants
J = 0.0002;               % Motor Inertia
L = 2.72e-3;              % Winding Self Inductance
M = -1.5e-3;              % Mutual Inductance
P = 4;                    % Number of poles
num_phases = 3;
Rs = 0.7;                 % Resistance per Phase (ohms)
turns_per_phase = 100;
L_r = 0.3;                % Rotor length (m)
R_r = 0.2;                % Rotor radius (m)

% NOT FILLED OUT WITH CORRECT NUMBERS YET
lambda_p = 1;
B = 1;

%% Derived Terms
L1 = L - M;

%% State space model
% Non-dynamic portion of the A matrix
A_base = ...
[-Rs/L1   0     0     0     0;
    0  -Rs/L1   0     0     0;
    0     0  -Rs/L1   0     0;
    0     0     0   -B/J    0;
    0     0     0    P/2    0];

B = ...
[1/L1  0    0   ;
  0   1/L1  0   ;
  0    0   1/L1];