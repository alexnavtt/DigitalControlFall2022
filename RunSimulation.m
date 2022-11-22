clc
clear
close all

%% Define constants
J  = 0.0002;                % Motor Inertia (kg*m^2)
L  = 1;                     % Winding Inductance (H)
R  = 5;                     % Motor Resistance (ohms)
b  = 0.5;                   % Damping constant (N*s)
Kt = 1.0;                   % Motor Torque constant (N*m/A)
Ke = 1.0;                   % Back EMF constant (V*s/rad)

tf = 30.0;                  % Simulation duration (s)


%% Define Controller Parameters

% PID parameters
P = 5;                        % Proportional Gain
I = 1;                        % Integral Gain
D = 0;                        % Derivative Gain

% Discretization (Not yet implemented)
Ts = 0.10;                      % Sample Time (s)

% Whether or not to include a disturbance step input in the simulation
add_disturbance = true;         % Whether or not to add a diturbance
disturbance_time = 10;           % Time at which to apply disturbance (s)
disturbance_amplitude = 0.1;    % Size of disturbance (Nm)

%% Create a reference input angular velocity to target

% Can be "custom", "constant_val" or "constant_acc"
input_type = uint8(InputType.constant_acc);

% Custom input signal
simin = struct();
simin.time = (0:0.01:tf)';
simin.signals.values = zeros(size(simin.time));

% Constant input signal
target_omega = 1;               % rad/s

% Constant input acceleration 
target_alpha = 0.1;             % rad/s^2
max_omega    = 1;               % rad/s

%% Run the simulation
results = sim('BrushlessMotorControlSim.slx');

%% Plot the results

[Tw, Yw] = getSimTimeSeries(results, 'omega');
[Tw_star, Yw_star] = getSimTimeSeries(results, 'setpoint');
plot(Tw, Yw, Tw_star, Yw_star)


%% Helper Functions

function [t, signal] = getSimTimeSeries(sim_output, name)

    time_series = sim_output.yout.getElement(name);
    t      = time_series.Values.Time;
    signal = time_series.Values.Data;

end