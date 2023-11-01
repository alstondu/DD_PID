% Controller designed for calculating the distance between the wheels

% PID controller to keep the car moving straight
e  = y(1)-y(2);
de = (e-e_prev)/dt;
ie = ie + e*dt;
u_turn = Kp_l_w*e + Ki_l_w*ie + Kd_l_w*de;
e_prev = e;

r = 0.1;    % Radius of the wheels
Length_left = (y(1) * 2 * pi * r) / 64;    % Distance travelled by the left wheel
Length_right = (y(2) * 2 * pi * r) / 64;    %Distance travelled by the right wheel
u_m = 4;

% Go straight When the robot travelled less than 1.5, otherwise, rotate 
if Length_right < 1.5 &&  Length_left < 1.5
    u = [u_m-u_turn; u_m+u_turn];
else
    u = [4,0];
end

% Average and record the two range sensor data on each side 
sensor_record_left(end+1) = (y(5)+y(6))/2;
sensor_record_right(end+1) = (y(7)+y(8))/2;

