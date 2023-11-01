% Analyse system with enumeration method

clc;
clear all;
close all;

% Initialize variables
% Transfer function of the system
Gm = tf(0.05, [0.1, 1, 0, 0]);
best_Kp = 1;
best_phase_margin=0;

% Set up the searching range of Kp
Kp_values = 0.1:0.01:20;


for Kp = Kp_values
    % Create the PD controller
    Gc = tf([10 *Kp, Kp], 1);
    
    % Calculate the phase margin
   [Amplitude, phase_margin, Wcg, Wcp] = margin(Gc*Gm)

    % Check whether a bigger phase margin is found
    if phase_margin > best_phase_margin
        best_Kp = Kp;
        best_phase_margin = phase_margin;
    end
end

% Print the maximum Kp and phase margin
disp(['Optimal Kp: ', num2str(best_Kp)]);
disp(['Max Phase Margin: ', num2str(best_phase_margin), ' degrees']);
