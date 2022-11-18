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
P = 1;                   % Proportional Gain
I = 4;                      % Integral Gain
D = 0;                      % Derivative Gain

% Discretization (Not yet implemented)
Ts = 0.10;                  % Sample Time (s)

%% Run the simulation
results = sim('BrushlessMotorControlSim.slx');

%% Plot the results

[Tw, Yw] = getSimTimeSeries(results, 'omega');
plot(Tw, Yw)


%% Helper Functions

function [t, signal] = getSimTimeSeries(sim_output, name)

    time_series = sim_output.yout.getElement(name);
    t      = time_series.Values.Time;
    signal = time_series.Values.Data;

end