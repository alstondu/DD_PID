% Scrips to initialize the parameters  of the controller_Q3

r = 0.1;    % Wheel radius
l_w = 0.25; % Distance between the wheels


% Initialize the parameters for the PID controller
e_prev = 0;
ie = 0;

Kp = 10;
Ki = 0.5;
Kd = 0;


% Initialize variables in controller_Q3.m
% Current coordinates and orientation
x_ = 0;
y_ = 0;
Phi = 0;

% Previouscoordinates and orientation
x_pre = 0;
y_pre = 0;
Phi_pre = 0;

% Variable used for state determination
flag = 0;

% Odometry values of last sampling
odo_left_pre = 0;
odo_right_pre = 0;

