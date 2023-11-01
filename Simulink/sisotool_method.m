% Analyse system with sisotool method

clc;
clear all;
close all;

%Transfer function of the system
G = tf(0.05 , [0.1, 1, 0, 0])

%Initialize a default Kp
Kp=1;  

% Create the PD controller
C = tf([10*Kp, Kp], 1)

sisotool(G,C)
