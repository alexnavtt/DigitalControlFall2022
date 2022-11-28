clc
clear
close all
%#ok<*UNRCH>

%% Define constants
% Datasheet: https://catalogue.precisionmicrodrives.com/product/datasheet/124-802-001-24mm-dc-motor-31mm-type-datasheet.pdf
J  = 0.0006;                % Motor Inertia (kg*m^2)
L  = 18e-3;                 % Winding Inductance (H)
R  = 11;                    % Motor Resistance (ohms)
b  = 1e-6;                  % Damping constant (N*s/rad)
Kt = 21.1e-3;               % Motor Torque constant (N*m/A)
Ke = 21.1e-3;               % Back EMF constant (V*s/rad)
V_max = 24;                 % Rated voltage (V)

Tf = 30.0;                  % Simulation duration (s)

% Discretization
Ts = 0.1;                   % Sample Time (s)
continuous = false;         % Whether or not to discretize the sim

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

% Determine which state we are trying to control
C = [0 0 0];
C(2) = 1;

%% Define Controller Parameters

% Decide what type of controller to use - can be "LQR" or "PID"
controller_type = uint8(ControllerType.LQR);

% PID parameters
P = 1;                          % Proportional Gain
I = 0;                          % Integral Gain
D = 0;                          % Derivative Gain

% LQR parameters
Q_lqr = ...
 [10    0     0  ;
  0     1     0  ;
  0     0    0.1];

R_lqr = 0.001;

K_integral = 10;

% Whether or not to include a disturbance step input in the simulation
add_disturbance = true;         % Whether or not to add a diturbance
disturbance_time = 10;          % Time at which to apply disturbance (s)
disturbance_duration = 10;      % Duration for which disturbance is applied (s)
disturbance_amplitude = 0.02;   % Size of disturbance (Nm)

%% Create a reference input angular velocity to target

% Can be "custom", "constant_val" or "constant_acc"
input_type = uint8(InputType.custom);

% Constant input signal
velocity_reference = 100;       % rad/s
position_reference = 2;         % rad
current_reference  = 1;         % A
constant_reference = C * [position_reference; velocity_reference; current_reference];

% Custom input signal
simin = struct();
simin.time = (0:0.01:Tf)';
simin.signals.values = constant_reference*sin(0.5*simin.time);

% Constant input acceleration 
target_alpha = 5;               % rad/s^2
max_omega    = 100;             % rad/s

%% Calculate the LQR gain
if ~C(1)
    % If we're not doing position control, make position have almost no weight
    Q_lqr(1,1) = 1e-5;
end

if continuous
    K_lqr = lqr(A, B(:,1), Q_lqr, R_lqr);
else
    ss_d = c2d(ss(A, B(:,1), C, 0), Ts, 'zoh');
    K_lqr = dlqr(ss_d.A, ss_d.B, Q_lqr, R_lqr);
end

if ~C(1)
    K_lqr(1) = 0;
end

%% Plot the results
global results
f1 = figure();
Ts_vec = [0.2, 0.1, 0.01, 0.001];
legend_cell = cell(1, numel(Ts_vec));

for i = 1:numel(Ts_vec)
    
    % Run the simulation
    Ts = Ts_vec(i);
    results = sim('BrushlessMotorControlSim.slx');

    % Plot the motor speed
    f1;

    subplot(2, 3, 1:3)
    hold on
    grid on
    xlabel("Time (s)")
    if C(1)
        [T, Y] = getSimTimeSeries('theta');
        Y = mod(Y + pi, 2*pi) - pi;
        Y = Y + 2*pi * (Y < -pi);
        ylabel("Motor Position (rad)")
        title("Motor Position Control")
    elseif C(2)
        [T, Y] = getSimTimeSeries('omega');
        ylabel("Motor Speed (rad/s)")
        title("Motor Speed Control")
    else
        [T, Y] = getSimTimeSeries('current');
        ylabel("Motor Current (A)")
        title("Motor Current Control")
    end
    plot(T, Y)

    % Plot the input
    subplot(2, 3, 4)
    hold on
    [Tv, Yv] = getSimTimeSeries('voltage');
    plot(Tv, Yv)
    xlabel("Time (s)")
    ylabel("V")
    ax = gca();
    ax.YLabel.Rotation = 0;
    title("Input Voltage")
    grid on

    % Plot the effort
    subplot(2, 3, 6)
    hold on
    [Ti, Yi] = getSimTimeSeries('current');
    plot(Ti, Yi)
    xlabel("Time (s)")
    ylabel("A")
    ax = gca();
    ax.YLabel.Rotation = 0;
    title("Motor Current")
    grid on
    
    legend_cell{i} = ['Ts = ' num2str(Ts)];

    % We only need to do this once if we're in the continuous version
    if continuous
        break
    end
end

% Plot the disturbance
subplot(2, 3, 5)
hold on
[T_tau, Y_tau] = getSimTimeSeries('torque');
plot(T_tau, Y_tau)
xlabel("Time (s)")
ylabel("Nm")
ax = gca();
ax.YLabel.Rotation = 0;
title("Disturbance Torque")
ylim([0 2*max(Y_tau)])
grid on

% Plot the setpoint
[T_setpoint, Y_setpoint] = getSimTimeSeries('setpoint');
f1;
subplot(2,3,1:3)
plot(T_setpoint, Y_setpoint)
f1.WindowState = 'Maximized';
legend_cell = [legend_cell "Setpoint"];
legend(legend_cell)

%% Helper Functions

function [t, signal] = getSimTimeSeries(name)

    global results

    time_series = results.logsout.getElement(name);
    t      = time_series.Values.Time;
    signal = time_series.Values.Data;

end