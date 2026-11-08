%% Parametric Sensitivity Analysis of the FSD Suspension
% This script conducts a parameter sweep on the Frequency-Selective 
% Damper (FSD) framework. It systematically iterates through distinct 
% configurations of baseline damping, FSD damping, and cutoff frequencies 
% to evaluate their isolated effects on ride quality (RMS acceleration).
%
% The analysis executes the 'fsd_suspension' Simulink model over a matrix 
% of test variables, extracting the structural response for comparative 
% visualization.

clc; clear; close all;

%% 1. Constant Vehicle & Environmental Parameters
% These parameters remain fixed throughout the sensitivity analysis.

% Mass and spring characteristics
assignin('base', 'ms', 300);    % Sprung mass (kg)
assignin('base', 'mu', 40);     % Unsprung mass (kg)
assignin('base', 'ks', 16000);  % Suspension stiffness (N/m)
assignin('base', 'kt', 190000); % Tyre stiffness (N/m)

% Simulation settings
assignin('base', 'Tsim', 50);
assignin('base', 'Ts', 0.0001);
assignin('base', 'vehicle_speed', 15);

m1=ms; m2=mu; k1=ks; k2=kt;
b2 = 100;              % Tyre damping (N.s/m)

% Standard Damper Parameter (Set to firm to match FSD low-frequency control)
b1 = 3500;          % Standard suspension damping (N.s/m)

%% 2. Definition of the Test Matrix
% Three distinct tuning scenarios are defined to observe the transition 
% from a comfort-biased setup to a firm, track-oriented setup.

% Scenario 1: Comfort / Soft Setup
% Scenario 2: Balanced / Standard Setup
% Scenario 3: Sport / Firm Setup
% Scenario 4: Extreme High-Frequency Cutoff

test_cases = struct();

test_cases(1).name = 'Comfort Focus (Low Damping, Low fc)';
test_cases(1).C_base = 1000;
test_cases(1).C_fsd = 1500;
test_cases(1).fc = 2;

test_cases(2).name = 'Balanced (Nominal Tuning)';
test_cases(2).C_base = 1500;
test_cases(2).C_fsd = 2000;
test_cases(2).fc = 3;

test_cases(3).name = 'Sport Focus (High Damping, High fc)';
test_cases(3).C_base = 2000;
test_cases(3).C_fsd = 3000;
test_cases(3).fc = 5;

test_cases(4).name = 'Aggressive Isolation (High Damping, Low fc)';
test_cases(4).C_base = 2000;
test_cases(4).C_fsd = 3000;
test_cases(4).fc = 1.5;

num_cases = length(test_cases);
rms_results = zeros(num_cases, 1);

%% 3. Automated Simulation Execution
% The routine loops through each test case, updates the workspace variables 
% governing the hydraulic stiffness and relaxation time, and extracts the RMS.

fprintf('--- Commencing FSD Parametric Sweep ---\n');

for i = 1:num_cases
    fprintf('Executing Iteration %d of %d: %s...\n', i, num_cases, test_cases(i).name);
    
    % Extract current test values
    current_C_base = test_cases(i).C_base;
    current_C_fsd = test_cases(i).C_fsd;
    current_fc = test_cases(i).fc;
    
    % Calculate derived hydraulic parameters (Zener mechanics)
    current_tau = 1 / (2 * pi * current_fc);
    current_K_fsd = current_C_fsd / current_tau; 
    
    % Inject parameters into the base workspace for Simulink
    assignin('base', 'C_base', current_C_base);
    assignin('base', 'C_fsd', current_C_fsd);
    assignin('base', 'K_fsd', current_K_fsd);
    assignin('base', 'tau', current_tau);
    assignin('base', 'fc', current_fc);
    
    % Execute model
    try
        simOut = sim('fsd_suspension', 'StopTime', num2str(evalin('base', 'Tsim')), 'SrcWorkspace', 'current');
        
        % Extract acceleration and calculate RMS
        accel_data = simOut.body_acc_fsd.signals.values;
        rms_results(i) = rms(accel_data - mean(accel_data)); % DC-offset corrected RMS
        
    catch ME
        fprintf('Error evaluating scenario %d: %s\n', i, ME.message);
        rms_results(i) = NaN;
    end
end

fprintf('--- Simulation Sweep Complete ---\n\n');

%% 4. Results Visualization
% The performance metrics are compiled into a comparative bar chart 
% detailing the ride harshness for each tuning profile.

figure('Name', 'FSD Parametric Analysis', 'Color', 'w', 'Position', [150, 150, 800, 500]);
bar_handle = bar(rms_results, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k', 'LineWidth', 1.2);

% Chart formatting
set(gca, 'XTick', 1:num_cases, 'XTickLabel', {test_cases.name}, 'FontSize', 10);
xtickangle(15);
ylabel('Sprung Mass RMS Acceleration (m/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
title('Parametric Sensitivity Analysis: FSD Tuning Profiles', 'FontSize', 13);
grid on;

% Add numerical labels atop the bars
for i = 1:num_cases
    if ~isnan(rms_results(i))
        text(i, rms_results(i) + 0.005, sprintf('%.4f', rms_results(i)), ...
            'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    end
end
