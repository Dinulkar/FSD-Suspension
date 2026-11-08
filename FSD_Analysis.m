%% Dynamic Analysis of Frequency Selective Damping (FSD) Systems
% A Numerical Case Study of the FSD Suspension Architecture
%
% This document presents a comprehensive numerical simulation and frequency 
% analysis of a Frequency Selective Damping (FSD) system. By employing a 
% Zener mechanical model, the proposed system effectively isolates 
% high-frequency road inputs while maintaining low-frequency roll and pitch 
% stability, circumventing the traditional compromise between ride comfort 
% and vehicle handling.

%% 1. Introduction and Mathematical Formulation
% Traditional passive dampers rely on a fixed force-velocity relationship. 
% Conversely, an FSD system utilizes a mechanical low-pass filter 
% (a secondary valve and hydraulic control chamber) to dynamically modulate 
% the damping coefficient based on the frequency of the suspension stroke.
%
% The total damping force is governed by a modified Zener model:
% F_total = C_base * x_dot + F_fsd
%
% Where the transient FSD force (F_fsd) is defined by the differential equation:
% d(F_fsd)/dt = K_fsd * x_dot - (K_fsd / C_fsd) * F_fsd

clc; clear; close all;

%% 2. Quarter-Car Model Parameters
% The physical parameters reflect a standard sport utility vehicle (SUV) 
% profile, allocating approximately one quarter of the vehicle's mass to a 
% single suspension strut.

ms = 300; m1=ms;     % Sprung mass (kg)
mu = 40;  m2=mu;     % Unsprung mass (kg)

ks = 16000; k1=ks;     % Suspension spring stiffness (N/m)
kt = 190000; k2=kt;    % Tyre stiffness (N/m)
b2 = 100;              % Tyre damping (N.s/m)

% Standard Damper Parameter (Set to firm to match FSD low-frequency control)
b1 = 3500;          % Standard suspension damping (N.s/m)

%% 3. Frequency Selective Damping (FSD) Parameters
% The FSD parameters govern the hydraulic chamber's response. At low 
% frequencies, the system yields a combined damping of 3500 N-s/m 
% (C_base + C_fsd). At high frequencies, the valve bypasses, dropping the 
% effective damping to 1500 N-s/m.

C_base = 1500;      % Baseline damping (N-s/m)
C_fsd = 2000;       % Additional FSD damping when closed (N-s/m)
K_fsd = 50000;      % Hydraulic stiffness of FSD control chamber (N/m)

% High-pass filter characteristics
fc = 3;             % Cutoff frequency (Hz)
tau = 1/(2*pi*fc);  % Relaxation time constant

%% 4. Simulation Initialization
% The simulation is conducted over a 50-second interval to capture 
% steady-state frequency responses.

Tsim = 50;          % Total simulation time (s)
Ts = 0.0001;        % Sample time (s)
vehicle_speed = 15; % Vehicle forward velocity (m/s)
g = 9.81;           % Gravitational acceleration (m/s^2)

%% 5. System Natural Frequencies
% The theoretical natural frequencies of the sprung and unsprung masses 
% are calculated to establish the operational bandwidths of interest.

f_body = (1/(2*pi))*sqrt(ks/ms);
f_wheel = (1/(2*pi))*sqrt((kt+ks)/mu);

fprintf('Body Natural Frequency (Sprung Mass): %.2f Hz\n', f_body);
fprintf('Wheel Hop Frequency (Unsprung Mass): %.2f Hz\n', f_wheel);
fprintf('FSD Cutoff Frequency: %.2f Hz\n\n', fc);

%% 6. Execution of the Simulink Model
% The numerical model 'fsd_suspension.slx' is executed. It is assumed the 
% model outputs 'body_acc_standard' and 'body_acc_fsd' to the workspace.
simOut = sim('fsd_suspension');

%% 7. Fast Fourier Transform (FFT) Analysis
% A frequency-domain analysis is conducted to evaluate the acceleration 
% amplitude transmitted to the sprung mass.

signals = {
    body_acc_standard.signals.values
    body_acc_fsd.signals.values
};

names = {'Standard Damper (Firm)','FSD Damper'};

figure('Name', 'FSD Frequency Analysis', 'Color', [1 1 1], 'Position', [100, 100, 800, 500]);
hold on; grid on;

for i = 1:2
    x = signals{i};
    x = x - mean(x); % Remove DC offset
    N = length(x);
    
    Y = fft(x);
    P = abs(Y/N);
    P = P(1:floor(N/2)+1); % Single-sided spectrum
    
    f = (0:floor(N/2))/(N*Ts);
    plot(f, P, 'LineWidth', 1.5);
end

% Human vibration sensitivity reference band (ISO 2631)
xline(4, '--r', '4 Hz (Sensitivity Start)', 'LabelVerticalAlignment', 'bottom');
xline(8, '--r', '8 Hz (Sensitivity End)', 'LabelVerticalAlignment', 'bottom');

xlim([0 25]);
xlabel('Frequency (Hz)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Acceleration Amplitude (m/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
title('Transmitted Acceleration: FSD vs. Standard Damper', 'FontSize', 12);
legend(names, 'Location', 'northeast');

%% 8. Root Mean Square (RMS) Ride Quality Evaluation
% The RMS acceleration is calculated to provide a singular metric for 
% ride harshness over the simulated road profile.

rms_std = rms(body_acc_standard.signals.values);
rms_fsd = rms(body_acc_fsd.signals.values);

fprintf('=== Ride Quality Metrics ===\n');
fprintf('Standard Damper RMS Acceleration: %.4f m/s^2\n', rms_std);
fprintf('FSD Damper RMS Acceleration:      %.4f m/s^2\n', rms_fsd);

if rms_fsd < rms_std
    fprintf('Result: The FSD system successfully reduced overall cabin acceleration.\n');
else
    fprintf('Result: The standard damper yielded lower overall acceleration for this specific profile.\n');
end

