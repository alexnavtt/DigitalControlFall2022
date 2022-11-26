clc
clear
close all
%#ok<*UNRCH>

%% Define constants
J  = 0.0006;                % Motor Inertia (kg*m^2)
L  = 18e-3;                 % Winding Inductance (H)
R  = 11;                    % Motor Resistance (ohms)
b  = 1e-6;                  % Damping constant (N*s/rad)
Kt = 21.1e-3;               % Motor Torque constant (N*m/A)
Ke = 21.1e-3;               % Back EMF constant (V*s/rad)
V_max = 24;                 % Rated voltage (V)

tf = 50.0;                  % Simulation duration (s)

% Discretization
Ts = 0.1;                   % Sample Time (s)
continuous = true;          % Whether or not to discretize the sim

% Note that the PID block has to be manually toggled between continuous and
% discrete

%% Define State Space
A = [0    1    0   ;
     0  -b/J  Kt/J ;
     0 -Ke/L  -R/L];

% First column is for voltage input
% Second column is for disturbance torque input
B = ...
[ 0    0  ;
  0  -1/J ;
 1/L   0 ];

% Make the state vector the output
C = eye(3);

%% Define Controller Parameters

% Decide what type of controller to use
controller_type = uint8(ControllerType.LQR);

% PID parameters
P = 1;                          % Proportional Gain
I = 0;                          % Integral Gain
D = 0;                          % Derivative Gain

% LQR parameters
Q_lqr = ...
[1e-4   0     0  ;
  0     1     0  ;
  0     0    0.1];

R_lqr = 0.0001;

K_integral = 50;

% Whether or not to include a disturbance step input in the simulation
add_disturbance = false;         % Whether or not to add a diturbance
disturbance_time = 10;          % Time at which to apply disturbance (s)
disturbance_amplitude = 0.05;    % Size of disturbance (Nm)

%% Create a reference input angular velocity to target

% Can be "custom", "constant_val" or "constant_acc"
input_type = uint8(InputType.custom);

% Custom input signal
simin = struct();
simin.time = (0:0.01:tf)';
simin.signals.values = 50*sin(simin.time);

% Constant input signal
target_omega = 100;             % rad/s

% Constant input acceleration 
target_alpha = 5;               % rad/s^2
max_omega    = 100;             % rad/s

%% Calculate the LQR gain
if continuous
    K_lqr = lqr(A, B(:,1), Q_lqr, R_lqr);
else
    ss_d = c2d(ss(A, B(:,1), C, 0), Ts);
    K_lqr = dlqr(ss_d.A, ss_d.B, Q_lqr, R_lqr);
end
K_lqr(1) = 0;

%% Run the simulation
results = sim('BrushlessMotorControlSim.slx');

%% Plot the results

% Plot the motor speed
f1 = figure();
f1.WindowState = 'Maximized';

subplot(2, 2, 1:2)
[Tw, Yw] = getSimTimeSeries(results, 'omega');
[Tw_star, Yw_star] = getSimTimeSeries(results, 'setpoint');
plot(Tw, Yw, Tw_star, Yw_star)
grid on
xlabel("Time (s)")
ylabel("Motor Speed (rad/s)")
title("Motor Speed Control")
legend("Motor Speed", "Setpoint", 'Location', "SouthEast")

% Plot the input
subplot(2, 2, 4)
[Tv, Yv] = getSimTimeSeries(results, 'voltage');
plot(Tv, Yv)
xlabel("Time (s)")
ylabel("V")
ax = gca();
ax.YLabel.Rotation = 0;
title("Input Voltage")
grid on

% Plot the effort
subplot(2, 2, 3)
[Ti, Yi] = getSimTimeSeries(results, 'current');
plot(Ti, Yi)
xlabel("Time (s)")
ylabel("A")
ax = gca();
ax.YLabel.Rotation = 0;
title("Motor Current")
grid on

%% Helper Functions

function [t, signal] = getSimTimeSeries(sim_output, name)

    time_series = sim_output.yout.getElement(name);
    t      = time_series.Values.Time;
    signal = time_series.Values.Data;

end