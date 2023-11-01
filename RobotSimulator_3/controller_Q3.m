% Controller for Q3

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
Phi_adjusted = mod(Phi, 2*pi);

% Determine the robot's current motion state
% When the robot passes the middle of  the obstacle left edge and its four corners
% the flag will be incremented by 1

% Variable to neutralise system uncertainty
shift= 0.05;

% The robot has reached the middle of  the obstacle left edge
if  (x_ >= 1.5- shift) && (x_pre < 1.5-shift) 
    flag = flag + 1

% The robot passes the bottom left corner    
elseif (y_ <= -1.5 + shift) && (y_pre > -1.5+ shift) 
    flag = flag + 1
    
% The robot passes the bottom left corner    
elseif (x_ >= 3.5 - shift) && (x_pre < 3.5- shift) 
    flag = flag + 1
    
% The robot passes the top right corner
elseif (y_ >= 1.5- shift) && (y_pre < 1.5- shift) 
    flag = flag + 1

% The robot passes the top left corner
elseif (x_ <= 1.5+ shift) && (x_pre > 1.5+ shift) 
    flag = flag + 1  

% The robot passes the middle of  the obstacle left edge
elseif (y_ <= shift) && (y_pre > shift) 
    flag = flag + 1

end

if flag == 0 % The robots is approching the obstcale
    x0 = 1.5; y0 = 0;
    [u_turn, e] = PID(x0, y0, x_, y_, Phi_adjusted, dt, e_prev,ie ,Kp, Ki, Kd);
    u_temp = [3-u_turn; 3+u_turn];
    u = max(-6, min(6, u_temp));
    e_prev = e;


elseif flag == 2 || flag == 7 
    x0 = 3.5; y0 = -1.5; % Heading for the bottom right corner
    [u_turn, e] = PID(x0, y0, x_, y_, Phi_adjusted, dt, e_prev,ie ,Kp, Ki, Kd);
    u_temp = [3-u_turn; 3+u_turn];
    u = max(-6, min(6, u_temp));
    e_prev = e;

elseif flag == 3 || flag == 8
    x0 = 3.5; y0 = 1.5; % Heading for the top right corner
    [u_turn, e] = PID(x0, y0, x_, y_, Phi_adjusted, dt, e_prev,ie ,Kp, Ki, Kd);
    u_temp = [3-u_turn; 3+u_turn];
    u = max(-6, min(6, u_temp));
    e_prev = e;

elseif flag == 4 || flag == 9
    x0 = 1.5; y0 = 1.5; % Heading for the top left corner
    [u_turn, e] = PID(x0, y0, x_, y_, Phi_adjusted, dt, e_prev,ie ,Kp, Ki, Kd);
    u_temp = [3-u_turn; 3+u_turn];
    u = max(-6, min(6, u_temp));
    e_prev = e;

elseif flag == 10 || flag == 5 || flag == 6 || flag == 1
    x0 = 1.5; y0 = -1.5; % Heading for the bottom left corner
    [u_turn, e] = PID(x0, y0, x_, y_, Phi_adjusted, dt, e_prev,ie ,Kp, Ki, Kd);
    u_temp = [3-u_turn; 3+u_turn];
    u = max(-6, min(6, u_temp));
    e_prev = e;

elseif flag == 11 
    if (Phi_adjusted >= 0) &&  (Phi_adjusted) <=0.15
         u = [0,0]; % If the orientation is 0 after arriving (0,0), the robot can stop
    else
         x0 = 0; y0 = 0; % Heading for the starting point
         [u_turn, e] = PID(x0, y0, x_, y_, Phi_adjusted, dt, e_prev,ie ,Kp, Ki, Kd);
         u_temp = [3-u_turn; 3+u_turn];
         u = max(-6, min(6, u_temp));
         e_prev = e;
    end
end

% Store current values as the previous values for the next execution
odo_left_pre = y(1);
odo_right_pre = y(2);

x_pre = x_;
y_pre = y_;
Phi_pre = Phi;

