function [u_turn, e] = PID(x0, y0, x_, y_, theta_adjusted, dt, e_prev,ie ,Kp, Ki, Kd)

    angle_rad = atan2(y0 - y_, x0 - x_);

    e = angle_rad - theta_adjusted;

    if e > pi
       e = e - 2*pi;
    elseif e < -pi
       e = e + 2*pi;
    end

    de = (e - e_prev) / dt;
    
    ie = ie + e * dt;
    
    u_turn = Kp * e + Ki * ie + Kd * de;
end
