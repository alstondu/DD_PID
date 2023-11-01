% Scrips to initialize the parameters for both of the controllers

% Initialize the parameters for the PID controller used to move straight
e_prev = 0;
ie = 0;

Kp_r = 1.25;
Ki_r = 0.25;
Kd_r = 0;

Kp_l_w = 2.5;
Ki_l_w = 0.5;
Kd_l_w = 0;

% Parameters for controller_r
% The range sensor and the odometry data when the obstacle is just sensed
d_sense = 0;
odo_left_sense = 0;
odo_right_sense = 0;

% The current distance of the car from the obstacle
% and that from the previous sampling period
d_front = 0;
d_front_pre = 0;

% Parameters for controller_l_w
% The average range sensor data on each side
sensor_record_left =[];
sensor_record_right = [];