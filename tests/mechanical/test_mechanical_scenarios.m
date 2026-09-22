%% Mechanical Plant Scenario Verification
% Model-Based Electric Drive Control & Virtual Verification Platform
%
% Scenarios:
%   M0 - Nominal baseline
%   M1 - Payload increase
%   M2 - External load disturbance
%
% Plant:
%   m*dv/dt = F_drive - F_load - b*v
%
% Verification:
%   Analytical first-order response vs Simulink response.

%% Test initialization

clearvars;
clc;

fprintf('\n');
fprintf('============================================================\n');
fprintf(' TEST: MECHANICAL PLANT SCENARIO VERIFICATION\n');
fprintf('============================================================\n');

%% Project paths

testFilePath = mfilename('fullpath');
testDirectory = fileparts(testFilePath);

projectRoot = fileparts(fileparts(testDirectory));

configDirectory = fullfile(projectRoot, 'config');
modelDirectory  = fullfile(projectRoot, 'models');

configFile = fullfile(configDirectory, 'init_project.m');

modelName = "agv_mechanical_plant";
modelFile = fullfile(modelDirectory, modelName + ".slx");

%% Validate files

if ~isfile(configFile)
    error('Configuration file not found:\n%s', configFile);
end

if ~isfile(modelFile)
    error('Simulink model not found:\n%s', modelFile);
end

addpath(configDirectory);
addpath(modelDirectory);

%% Load project configuration

run(configFile);

%% Load model

load_system(modelFile);

%% Common drivetrain quantities

driveForce = ...
    cfg.test.phase2.motor_torque * ...
    cfg.drivetrain.gear_ratio * ...
    cfg.drivetrain.efficiency / ...
    cfg.agv.wheel_radius;

resistanceCoefficient = ...
    cfg.agv.resistance_coefficient;

torqueStepTime = ...
    cfg.test.phase2.torque_step_time;

loadStepTime = ...
    cfg.test.phase2.load_step_time;

%% Verification settings

verification.velocityTolerance = 1e-3;       % [m/s]
verification.accelerationTolerance = 1e-3;   % [m/s^2]

verification.standardTime = 2.5;             % [s]
verification.preLoadTime = 1.5;              % [s]
verification.postLoadTime = 3.0;             % [s]

%% ============================================================
% M0 - NOMINAL BASELINE
% =============================================================

fprintf('\n');
fprintf('------------------------------------------------------------\n');
fprintf(' M0 - NOMINAL BASELINE\n');
fprintf('------------------------------------------------------------\n');

cfg.agv.mass = 200.0;                        % [kg]
cfg.test.phase2.load_force = 0.0;            % [N]

massM0 = cfg.agv.mass;
tauM0 = massM0 / resistanceCoefficient;

steadyVelocityM0 = ...
    driveForce / resistanceCoefficient;

elapsedM0 = ...
    verification.standardTime - torqueStepTime;

expectedVelocityM0 = ...
    steadyVelocityM0 * ...
    (1 - exp(-elapsedM0 / tauM0));

expectedAccelerationM0 = ...
    (driveForce / massM0) * ...
    exp(-elapsedM0 / tauM0);

simOutM0 = sim(modelName);

speedLogM0 = simOutM0.get('speed_log');
accelLogM0 = simOutM0.get('accel_log');

measuredVelocityM0 = interp1( ...
    speedLogM0.Time, ...
    speedLogM0.Data, ...
    verification.standardTime, ...
    'linear');

measuredAccelerationM0 = interp1( ...
    accelLogM0.Time, ...
    accelLogM0.Data, ...
    verification.standardTime, ...
    'linear');

velocityErrorM0 = ...
    abs(measuredVelocityM0 - expectedVelocityM0);

accelerationErrorM0 = ...
    abs(measuredAccelerationM0 - expectedAccelerationM0);

passM0 = ...
    velocityErrorM0 <= verification.velocityTolerance && ...
    accelerationErrorM0 <= verification.accelerationTolerance;

fprintf('Vehicle mass             : %10.2f kg\n', massM0);
fprintf('Time constant            : %10.4f s\n', tauM0);
fprintf('Verification time        : %10.2f s\n', verification.standardTime);

fprintf('\nVelocity\n');
fprintf('Expected                 : %10.4f m/s\n', expectedVelocityM0);
fprintf('Measured                 : %10.4f m/s\n', measuredVelocityM0);
fprintf('Absolute error           : %10.6f m/s\n', velocityErrorM0);

fprintf('\nAcceleration\n');
fprintf('Expected                 : %10.4f m/s^2\n', expectedAccelerationM0);
fprintf('Measured                 : %10.4f m/s^2\n', measuredAccelerationM0);
fprintf('Absolute error           : %10.6f m/s^2\n', accelerationErrorM0);

fprintf('\nResult                   : %s\n', passText(passM0));

%% ============================================================
% M1 - PAYLOAD INCREASE
% =============================================================

fprintf('\n');
fprintf('------------------------------------------------------------\n');
fprintf(' M1 - PAYLOAD INCREASE\n');
fprintf('------------------------------------------------------------\n');

cfg.agv.mass = 300.0;                        % [kg]
cfg.test.phase2.load_force = 0.0;            % [N]

massM1 = cfg.agv.mass;
tauM1 = massM1 / resistanceCoefficient;

steadyVelocityM1 = ...
    driveForce / resistanceCoefficient;

elapsedM1 = ...
    verification.standardTime - torqueStepTime;

expectedVelocityM1 = ...
    steadyVelocityM1 * ...
    (1 - exp(-elapsedM1 / tauM1));

expectedAccelerationM1 = ...
    (driveForce / massM1) * ...
    exp(-elapsedM1 / tauM1);

simOutM1 = sim(modelName);

speedLogM1 = simOutM1.get('speed_log');
accelLogM1 = simOutM1.get('accel_log');

measuredVelocityM1 = interp1( ...
    speedLogM1.Time, ...
    speedLogM1.Data, ...
    verification.standardTime, ...
    'linear');

measuredAccelerationM1 = interp1( ...
    accelLogM1.Time, ...
    accelLogM1.Data, ...
    verification.standardTime, ...
    'linear');

velocityErrorM1 = ...
    abs(measuredVelocityM1 - expectedVelocityM1);

accelerationErrorM1 = ...
    abs(measuredAccelerationM1 - expectedAccelerationM1);

passM1 = ...
    velocityErrorM1 <= verification.velocityTolerance && ...
    accelerationErrorM1 <= verification.accelerationTolerance;

fprintf('Vehicle mass             : %10.2f kg\n', massM1);
fprintf('Time constant            : %10.4f s\n', tauM1);
fprintf('Verification time        : %10.2f s\n', verification.standardTime);

fprintf('\nVelocity\n');
fprintf('Expected                 : %10.4f m/s\n', expectedVelocityM1);
fprintf('Measured                 : %10.4f m/s\n', measuredVelocityM1);
fprintf('Absolute error           : %10.6f m/s\n', velocityErrorM1);

fprintf('\nAcceleration\n');
fprintf('Expected                 : %10.4f m/s^2\n', expectedAccelerationM1);
fprintf('Measured                 : %10.4f m/s^2\n', measuredAccelerationM1);
fprintf('Absolute error           : %10.6f m/s^2\n', accelerationErrorM1);

fprintf('\nResult                   : %s\n', passText(passM1));

%% ============================================================
% M2 - EXTERNAL LOAD DISTURBANCE
% =============================================================

fprintf('\n');
fprintf('------------------------------------------------------------\n');
fprintf(' M2 - EXTERNAL LOAD DISTURBANCE\n');
fprintf('------------------------------------------------------------\n');

cfg.agv.mass = 200.0;                        % [kg]
cfg.test.phase2.load_force = 100.0;          % [N]

massM2 = cfg.agv.mass;
loadForceM2 = cfg.test.phase2.load_force;

tauM2 = ...
    massM2 / resistanceCoefficient;

%% Response before load disturbance

steadyVelocityBeforeLoad = ...
    driveForce / resistanceCoefficient;

elapsedPreLoad = ...
    verification.preLoadTime - torqueStepTime;

expectedVelocityPreLoad = ...
    steadyVelocityBeforeLoad * ...
    (1 - exp(-elapsedPreLoad / tauM2));

expectedAccelerationPreLoad = ...
    (driveForce / massM2) * ...
    exp(-elapsedPreLoad / tauM2);

%% Velocity exactly when load disturbance begins

elapsedToLoadStep = ...
    loadStepTime - torqueStepTime;

velocityAtLoadStep = ...
    steadyVelocityBeforeLoad * ...
    (1 - exp(-elapsedToLoadStep / tauM2));

%% Response after load disturbance

steadyVelocityAfterLoad = ...
    (driveForce - loadForceM2) / ...
    resistanceCoefficient;

elapsedPostLoad = ...
    verification.postLoadTime - loadStepTime;

expectedVelocityPostLoad = ...
    steadyVelocityAfterLoad + ...
    (velocityAtLoadStep - steadyVelocityAfterLoad) * ...
    exp(-elapsedPostLoad / tauM2);

expectedAccelerationPostLoad = ...
    ( ...
    driveForce - ...
    loadForceM2 - ...
    resistanceCoefficient * expectedVelocityPostLoad ...
    ) / massM2;

%% Simulate M2

simOutM2 = sim(modelName);

speedLogM2 = simOutM2.get('speed_log');
accelLogM2 = simOutM2.get('accel_log');

%% Extract pre-load response

measuredVelocityPreLoad = interp1( ...
    speedLogM2.Time, ...
    speedLogM2.Data, ...
    verification.preLoadTime, ...
    'linear');

measuredAccelerationPreLoad = interp1( ...
    accelLogM2.Time, ...
    accelLogM2.Data, ...
    verification.preLoadTime, ...
    'linear');

%% Extract post-load response

measuredVelocityPostLoad = interp1( ...
    speedLogM2.Time, ...
    speedLogM2.Data, ...
    verification.postLoadTime, ...
    'linear');

measuredAccelerationPostLoad = interp1( ...
    accelLogM2.Time, ...
    accelLogM2.Data, ...
    verification.postLoadTime, ...
    'linear');

%% Errors

velocityErrorPreLoad = ...
    abs(measuredVelocityPreLoad - expectedVelocityPreLoad);

accelerationErrorPreLoad = ...
    abs(measuredAccelerationPreLoad - expectedAccelerationPreLoad);

velocityErrorPostLoad = ...
    abs(measuredVelocityPostLoad - expectedVelocityPostLoad);

accelerationErrorPostLoad = ...
    abs(measuredAccelerationPostLoad - expectedAccelerationPostLoad);

passM2Pre = ...
    velocityErrorPreLoad <= verification.velocityTolerance && ...
    accelerationErrorPreLoad <= verification.accelerationTolerance;

passM2Post = ...
    velocityErrorPostLoad <= verification.velocityTolerance && ...
    accelerationErrorPostLoad <= verification.accelerationTolerance;

passM2 = passM2Pre && passM2Post;

%% M2 report

fprintf('Vehicle mass             : %10.2f kg\n', massM2);
fprintf('External load step       : %10.2f N\n', loadForceM2);
fprintf('Load application time    : %10.2f s\n', loadStepTime);

fprintf('\nBefore Load @ %.2f s\n', verification.preLoadTime);

fprintf('Expected velocity        : %10.4f m/s\n', ...
    expectedVelocityPreLoad);

fprintf('Measured velocity        : %10.4f m/s\n', ...
    measuredVelocityPreLoad);

fprintf('Expected acceleration    : %10.4f m/s^2\n', ...
    expectedAccelerationPreLoad);

fprintf('Measured acceleration    : %10.4f m/s^2\n', ...
    measuredAccelerationPreLoad);

fprintf('Result                   : %s\n', ...
    passText(passM2Pre));

fprintf('\nAfter Load @ %.2f s\n', verification.postLoadTime);

fprintf('Expected velocity        : %10.4f m/s\n', ...
    expectedVelocityPostLoad);

fprintf('Measured velocity        : %10.4f m/s\n', ...
    measuredVelocityPostLoad);

fprintf('Expected acceleration    : %10.4f m/s^2\n', ...
    expectedAccelerationPostLoad);

fprintf('Measured acceleration    : %10.4f m/s^2\n', ...
    measuredAccelerationPostLoad);

fprintf('Result                   : %s\n', ...
    passText(passM2Post));

fprintf('\nM2 Overall Result        : %s\n', passText(passM2));

%% ============================================================
% FINAL SCENARIO SUMMARY
% =============================================================

testResult.M0Pass = passM0;
testResult.M1Pass = passM1;
testResult.M2Pass = passM2;

testResult.overallPass = ...
    passM0 && passM1 && passM2;

fprintf('\n');
fprintf('============================================================\n');
fprintf(' MECHANICAL SCENARIO SUMMARY\n');
fprintf('============================================================\n');

fprintf('M0 Nominal baseline       : %s\n', passText(passM0));
fprintf('M1 Payload increase       : %s\n', passText(passM1));
fprintf('M2 Load disturbance       : %s\n', passText(passM2));

fprintf('------------------------------------------------------------\n');
fprintf('OVERALL RESULT            : %s\n', ...
    passText(testResult.overallPass));

fprintf('============================================================\n\n');

%% Close model

close_system(modelName, 0);

%% Automated assertion

assert( ...
    testResult.overallPass, ...
    'MechanicalScenarioTest:VerificationFailed', ...
    'One or more mechanical scenarios failed verification.');

%% Local utility

function text = passText(condition)

    if condition
        text = 'PASS';
    else
        text = 'FAIL';
    end

end