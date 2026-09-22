%% Model-Based Electric Drive Control & Virtual Verification Platform
% Project configuration
%
% Purpose:
% Defines centralized configuration parameters for the electric-drive
% virtual verification platform.
%
% All project parameters are stored inside the cfg structure to prevent
% workspace pollution and naming conflicts with MATLAB functions.

%% Initialize project configuration
% Rebuild the configuration structure from scratch on every execution.
% This prevents obsolete fields from previous configurations remaining
% in the MATLAB workspace.

cfg = struct();

%% Simulation configuration

cfg.simulation.stop_time = 5.0;                  % [s]

%% AGV configuration

cfg.agv.mass = 200.0;                            % [kg]
cfg.agv.wheel_radius = 0.10;                     % [m]

% Linearized longitudinal mechanical resistance.
% Engineering assumption used for model verification; not identified
% from a specific physical AGV.

cfg.agv.resistance_coefficient = 50.0;           % [N*s/m]

%% Drivetrain configuration

cfg.drivetrain.gear_ratio = 10.0;                % [-]
cfg.drivetrain.efficiency = 0.95;                % [-]

%% Reference operating point

cfg.reference.vehicle_speed = 1.50;              % [m/s]

%% Derived reference quantities

cfg.reference.wheel_speed_rad_s = ...
    cfg.reference.vehicle_speed / ...
    cfg.agv.wheel_radius;

cfg.reference.motor_speed_rad_s = ...
    cfg.reference.wheel_speed_rad_s * ...
    cfg.drivetrain.gear_ratio;

cfg.reference.motor_speed_rpm = ...
    cfg.reference.motor_speed_rad_s * ...
    60 / (2*pi);

%% Phase 2 baseline mechanical test configuration

cfg.test.phase2.motor_torque = 4.0;              % [N*m]
cfg.test.phase2.torque_step_time = 0.5;          % [s]

cfg.test.phase2.load_force = 0.0;                % [N]
cfg.test.phase2.load_step_time = 2.0;            % [s]

%% Phase 2 analytical drivetrain reference

cfg.test.phase2.expected_wheel_torque = ...
    cfg.test.phase2.motor_torque * ...
    cfg.drivetrain.gear_ratio * ...
    cfg.drivetrain.efficiency;

cfg.test.phase2.expected_drive_force = ...
    cfg.test.phase2.expected_wheel_torque / ...
    cfg.agv.wheel_radius;

%% Phase 2 first-order mechanical analytical quantities

cfg.test.phase2.initial_acceleration = ...
    (cfg.test.phase2.expected_drive_force - ...
    cfg.test.phase2.load_force) / ...
    cfg.agv.mass;

cfg.test.phase2.mechanical_time_constant = ...
    cfg.agv.mass / ...
    cfg.agv.resistance_coefficient;

cfg.test.phase2.steady_state_velocity = ...
    (cfg.test.phase2.expected_drive_force - ...
    cfg.test.phase2.load_force) / ...
    cfg.agv.resistance_coefficient;

%% Configuration summary

fprintf('\n');
fprintf('====================================================\n');
fprintf(' ELECTRIC DRIVE V&V PLATFORM - CONFIGURATION\n');
fprintf('====================================================\n');

fprintf('AGV mass              : %8.2f kg\n', ...
    cfg.agv.mass);

fprintf('Wheel radius          : %8.3f m\n', ...
    cfg.agv.wheel_radius);

fprintf('Resistance coefficient: %8.2f N*s/m\n', ...
    cfg.agv.resistance_coefficient);

fprintf('Gear ratio            : %8.2f : 1\n', ...
    cfg.drivetrain.gear_ratio);

fprintf('Drivetrain efficiency : %8.2f %%\n', ...
    cfg.drivetrain.efficiency * 100);

fprintf('Vehicle reference     : %8.2f m/s\n', ...
    cfg.reference.vehicle_speed);

fprintf('Motor reference       : %8.2f rpm\n', ...
    cfg.reference.motor_speed_rpm);

fprintf('Mechanical time const.: %8.2f s\n', ...
    cfg.test.phase2.mechanical_time_constant);

fprintf('Steady-state velocity : %8.2f m/s\n', ...
    cfg.test.phase2.steady_state_velocity);

fprintf('====================================================\n\n');