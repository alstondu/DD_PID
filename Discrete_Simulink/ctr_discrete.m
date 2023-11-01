% Script to discretise the PD controller

clc;
clear all;
close all;

% Continuous PD controller transfer function in Q5
C = tf([20, 2], 1)

% Discretise the PD controller
ctr_d = c2d(C, 0.02, 'tustin')