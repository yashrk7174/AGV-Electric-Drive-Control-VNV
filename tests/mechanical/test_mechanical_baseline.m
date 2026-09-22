%% Mechanical Plant Baseline Verification
% Model-Based Electric Drive Control & Virtual Verification Platform
%
% Purpose:
% Verify the AGV mechanical plant including velocity-dependent
% mechanical resistance against its analytical first-order solution.
%
% Plant equation:
%
%   m*dv/dt = F_drive - b*v
%
% Verification:
%   1. Vehicle velocity at a defined time
%   2. Vehicle acceleration at the same time
%
% Required logged outputs:
%   accel_log   - vehicle acceleration [m/s^2]
%   speed_log   - vehicle velocity [m/s]

%% Test initialization

clearvars;
clc;

fprintf('\n');
fprintf('============================================================\n');
fprintf(' TEST: MECHANICAL PLANT BASELINE VERIFICATION\n');
fprintf('============================================================\n');

%% Determine project paths

testFilePath = mfilename('fullpath');
testDirectory = fileparts(testFilePath);

projectRoot = fileparts(fileparts(testDirectory));

configDirectory = fullfile(projectRoot, 'config');
modelDirectory  = fullfile(projectRoot, 'models');

configFile = fullfile(configDirectory, 'init_project.m');

modelName = "agv_mechanical_plant";
modelFile = fullfile(modelDirectory, modelName + ".slx");

%% Validate required project files

if ~isfile(configFile)
    error( ...
        'MechanicalTest:MissingConfiguration', ...
        'Configuration file not found:\n%s', ...
        configFile);
end

if ~isfile(modelFile)
    error( ...
        'MechanicalTest:MissingModel', ...
        'Simulink model not found:\n%s', ...
        modelFile);
end

%% Add required project directories

addpath(configDirectory);
addpath(modelDirectory);

%% Load project configuration

run(configFile);

if ~exist('cfg', 'var')
    error( ...
        'MechanicalTest:MissingConfigurationStructure', ...
        'init_project.m did not create the required cfg structure.');
end

%% Extract baseline parameters

motorTorque = cfg.test.phase2.motor_torque;

gearRatio = cfg.drivetrain.gear_ratio;

drivetrainEfficiency = ...
    cfg.drivetrain.efficiency;

wheelRadius = cfg.agv.wheel_radius;

vehicleMass = cfg.agv.mass;

resistanceCoefficient = ...
    cfg.agv.resistance_coefficient;

torqueStepTime = ...
    cfg.test.phase2.torque_step_time;

externalLoad = ...
    cfg.test.phase2.load_force;

%% Analytical drivetrain calculations

expectedWheelTorque = ...
    motorTorque * ...
    gearRatio * ...
    drivetrainEfficiency;

expectedDriveForce = ...
    expectedWheelTorque / ...
    wheelRadius;

%% Analytical first-order mechanical model

mechanicalTimeConstant = ...
    vehicleMass / ...
    resistanceCoefficient;

steadyStateVelocity = ...
    (expectedDriveForce - externalLoad) / ...
    resistanceCoefficient;

initialAcceleration = ...
    (expectedDriveForce - externalLoad) / ...
    vehicleMass;

%% Verification settings

verification.time = 2.5;                     % [s]

verification.velocityTolerance = 1e-3;       % [m/s]
verification.accelerationTolerance = 1e-3;   % [m/s^2]

elapsedTime = ...
    verification.time - torqueStepTime;

if elapsedTime <= 0
    error( ...
        'MechanicalTest:InvalidVerificationTime', ...
        'Verification time must occur after the torque step.');
end

%% Analytical expected response

expectedVelocity = ...
    steadyStateVelocity * ...
    (1 - exp(-elapsedTime / mechanicalTimeConstant));

expectedAcceleration = ...
    initialAcceleration * ...
    exp(-elapsedTime / mechanicalTimeConstant);

%% Run simulation

load_system(modelFile);

fprintf('\nRunning model: %s\n', modelName);

simOut = sim(modelName);

%% Retrieve logged signals

accelLog = simOut.get('accel_log');
speedLog = simOut.get('speed_log');

%% Validate logged signal formats

if ~isa(accelLog, 'timeseries')
    close_system(modelName, 0);

    error( ...
        'MechanicalTest:InvalidAccelerationFormat', ...
        '"accel_log" must use Timeseries format.');
end

if ~isa(speedLog, 'timeseries')
    close_system(modelName, 0);

    error( ...
        'MechanicalTest:InvalidSpeedFormat', ...
        '"speed_log" must use Timeseries format.');
end

%% Extract simulated values at verification time

measuredVelocity = interp1( ...
    speedLog.Time, ...
    speedLog.Data, ...
    verification.time, ...
    'linear');

measuredAcceleration = interp1( ...
    accelLog.Time, ...
    accelLog.Data, ...
    verification.time, ...
    'linear');

%% Calculate verification errors

velocityAbsError = ...
    abs(measuredVelocity - expectedVelocity);

accelerationAbsError = ...
    abs(measuredAcceleration - expectedAcceleration);

%% PASS / FAIL evaluation

velocityPass = ...
    velocityAbsError <= ...
    verification.velocityTolerance;

accelerationPass = ...
    accelerationAbsError <= ...
    verification.accelerationTolerance;

testResult.velocityPass = velocityPass;
testResult.accelerationPass = accelerationPass;

testResult.overallPass = ...
    testResult.velocityPass && ...
    testResult.accelerationPass;

%% Engineering report

fprintf('\n');
fprintf('---------------- ANALYTICAL REFERENCE ----------------\n');

fprintf('Motor torque             : %10.4f N*m\n', ...
    motorTorque);

fprintf('Gear ratio               : %10.4f : 1\n', ...
    gearRatio);

fprintf('Drivetrain efficiency    : %10.2f %%\n', ...
    drivetrainEfficiency * 100);

fprintf('Wheel torque             : %10.4f N*m\n', ...
    expectedWheelTorque);

fprintf('Drive force              : %10.4f N\n', ...
    expectedDriveForce);

fprintf('Vehicle mass             : %10.4f kg\n', ...
    vehicleMass);

fprintf('Resistance coefficient   : %10.4f N*s/m\n', ...
    resistanceCoefficient);

fprintf('Mechanical time constant : %10.4f s\n', ...
    mechanicalTimeConstant);

fprintf('Steady-state velocity    : %10.4f m/s\n', ...
    steadyStateVelocity);

fprintf('Initial acceleration     : %10.4f m/s^2\n', ...
    initialAcceleration);

%% Velocity verification

fprintf('\n');
fprintf('----------------- VELOCITY CHECK ---------------------\n');

fprintf('Verification time        : %10.2f s\n', ...
    verification.time);

fprintf('Expected velocity        : %10.4f m/s\n', ...
    expectedVelocity);

fprintf('Measured velocity        : %10.4f m/s\n', ...
    measuredVelocity);

fprintf('Absolute error           : %10.6f m/s\n', ...
    velocityAbsError);

fprintf('Tolerance                : %10.6f m/s\n', ...
    verification.velocityTolerance);

if velocityPass
    fprintf('Result                   : PASS\n');
else
    fprintf('Result                   : FAIL\n');
end

%% Acceleration verification

fprintf('\n');
fprintf('--------------- ACCELERATION CHECK -------------------\n');

fprintf('Verification time        : %10.2f s\n', ...
    verification.time);

fprintf('Expected acceleration    : %10.4f m/s^2\n', ...
    expectedAcceleration);

fprintf('Measured acceleration    : %10.4f m/s^2\n', ...
    measuredAcceleration);

fprintf('Absolute error           : %10.6f m/s^2\n', ...
    accelerationAbsError);

fprintf('Tolerance                : %10.6f m/s^2\n', ...
    verification.accelerationTolerance);

if accelerationPass
    fprintf('Result                   : PASS\n');
else
    fprintf('Result                   : FAIL\n');
end

%% Final result

fprintf('\n');
fprintf('============================================================\n');

if testResult.overallPass
    fprintf(' OVERALL TEST RESULT : PASS\n');
else
    fprintf(' OVERALL TEST RESULT : FAIL\n');
end

fprintf('============================================================\n\n');

%% Close model

close_system(modelName, 0);

%% Automated assertion

assert( ...
    testResult.overallPass, ...
    'MechanicalTest:VerificationFailed', ...
    ['Mechanical plant baseline verification failed. ' ...
     'Review the reported errors above.']);