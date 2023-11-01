% Controller of Q2

% Usethe variables odo_left_pre  and odo_right_pre to record the previous values of y(1) and y(2).
% By subtracting these from the current y(1) and y(2), get the change of encoder for both wheels in one dt.  
delta_odo_left = y(1)-odo_left_pre;
delta_odo_right = y(2)-odo_right_pre;
delta_d_left = (delta_odo_left / 64) * (2 * pi * r);
delta_d_right = (delta_odo_right / 64) * (2 * pi * r);

% Calculate the distance moved and the offset angle of the robot within dt
delta_d_m = (delta_d_left + delta_d_right) / 2;
delta_Phi = (delta_d_right - delta_d_left) / l_w;

% Based on the calculated movement distance and angle change, 
% update the robot's coordinates and orientation

x_ = x_pre + delta_d_m * cos(Phi + 0.5 * delta_Phi);
y_ = y_pre + delta_d_m * sin(Phi + 0.5 * delta_Phi);
Phi = Phi_pre + delta_Phi;

%Use the flag to determine whether to return
if  (y_ < -0.03) && (y_pre >= -0.03) 
    flag = flag + 1;
end

if flag == 3 
    % When flag=3, it indicates that the robot has passed the left side of the obstacle twice and can return.
    Phi_adjusted = mod(Phi, 2*pi);
    if (Phi_adjusted >= 0) &&  (Phi_adjusted) <=0.15
            u = [0,0];
    else
            Phi_adjusted = mod(Phi, 2*pi);
            x0 = 0;
            y0 = 0;
            %Set the robot's target value to the initial point: (0,0)
            angle_rad = atan2(y0 - y_, x0 - x_);
            e = angle_rad - Phi_adjusted;
            if e > pi
               e = e - 2*pi;
            elseif e < -pi
               e = e + 2*pi;
            end
            u_turn = PID(e, dt, e_prev,ie ,Kp, Ki, Kd);
            u_temp = [5-u_turn; 5+u_turn];
            u = max(-6, min(6, u_temp));    %Input voltage -6~6 V
            e_prev = e;

    end

% If the robot's left-side sensors distance larger than 1.09 
% indicating no obstacle. And the front sensors distance larger than 0.5 from an obstacle 
% implies the robot is approaching the obstacle
elseif all(y(5:6) > 1.09) && all(y(3:4) > 0.5 ) 
    e = y(1) - y(2);
    u_turn = PID(e, dt, e_prev,ie ,Kp, Ki, Kd);
    u_temp = [4-u_turn; 4+u_turn];
    u = max(-6, min(6, u_temp));
    e_prev = e;

elseif all(y(3:4) > 1 ) 
    %The robot did not detect obstacles in front, indicating that the robot is circling the obstacle
    e1 = y(5)-y(6); % The difference in distance between the two sensors to the wall
    e2 = ((y(5)+y(6))/2)-0.5;   % The difference between the robot's distance to the obstacle and the target distance of 0.5 m
    e = e1 + e2;

    u_turn = PID(e, dt, e_prev,ie ,Kp, Ki, Kd);
    u_temp = [4-u_turn; 4+u_turn];
    u = max(-6, min(6, u_temp));
    e_prev = e;

else
    u = [6, 0];
    % If the robot doesn't fit the above scenarios
    % it's transitioning between "approaching the obstacle" and “wall-following procedure”, requiring make a turn
end

% Store current values as the previous values for the next execution
odo_left_pre = y(1);
odo_right_pre = y(2);

x_pre = x_;
y_pre = y_;
Phi_pre = Phi;

