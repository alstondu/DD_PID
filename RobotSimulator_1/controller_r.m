% Controller designed for calculating the wheel radius

% PID controller to keep the car moving straight
e  = y(1)-y(2);
de = (e-e_prev)/dt;
ie = ie + e*dt;

u_turn = Kp_r*e + Ki_r*ie + Kd_r*de;

u_m = 2;
u = [u_m-u_turn; u_m+u_turn];

e_prev = e;

% Calculate the current distance of the car from the obstacle in front of
% it by averaging the two data captured by the range sensors in the front
d_front = (y(3) + y(4))/2;

% Record the two odometry value and the distance 
% once the obstacle is within the range of the range sensors in the front 
if d_front < 1.09 && d_front_pre > 1.09
    odo_left_sense = y(1);
    odo_right_sense = y(2);
    d_sense = d_front;
end

% Store the current distance sensor data as distance_front_pre
d_front_pre = d_front;