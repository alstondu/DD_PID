% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 20;
dt          = 0.02;
TIME_SCALE  = 0.1; % speed up the simulation

% Initialise plot
figure;
ax1 = axes;
hold(ax1,'on');
axis('equal')
axis([-0.4 4.6 -2 2])
axis('manual')
xlabel('x');
ylabel('y');
ax1.Toolbar.Visible = 'off';
ax1.Interactions = [];

% Initialise Simulation
robot = DifferentialDriveWithObstacles(ax1);
robot.setState(zeros(9,1));
robot.setInput([0;0]);
robot.updateOutput;
csim = ControlSimulator(robot,TOTAL_TIME,dt);

trajplot = plot(csim.Log.Output(2,:),csim.Log.Output(3,:),'linewidth',1,'Color','k');
robot.plot;


%%%%%%%%%%%%%%%%%%%%%%%% IMPLEMENT THIS SCRIPT %%%%%%%%%%%%%%%%%%%%%%%%
init_Q1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = nan(1,csim.TotalSteps-1);
% Run Simulation
for i = 2:csim.TotalSteps
    tic
    
    % Read odometry and range sensors
    % y(1:2) - odometry
    % y(3:11) - range sensors
    
    y = robot.Output(4:11);
    
    % Control: determine dc motor inputs. assume controller only has access
    % to the odometry measurements variable 'y' and
    
    %%%%%%%%%%%%%%%%%%%%%%%% IMPLEMENT THIS SCRIPT %%%%%%%%%%%%%%%%%%%%%%%%
    % Use controller_r to calculate the wheel radius
    controller_r;

    % Use controller_l_w to calculate the distance between the wheels 
    % controller_l_w;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Simulation
    csim.step(u);
    set(trajplot,'XData',csim.Log.Output(2,:));
    set(trajplot,'YData',csim.Log.Output(3,:));
    robot.plot;
    drawnow nocallbacks limitrate
    time(i-1) = toc;
    pause(TIME_SCALE*dt-toc); 
end

% Run controller_r above to calculate the wheel radius
% The distance travelled after the obstacle is firstly sensed
d_L          =  d_sense - d_front;
d_R          = d_L;
n_ticks      = 64;
% Change of the odometry values after the obstacle is firstly sensed
delta_tick_L = y(1) - odo_left_sense;
delta_tick_R = y(2) - odo_right_sense;
% Calculate the radius based on the distance and the odometry values
r_L = (n_ticks*d_L)/(2*pi*delta_tick_L) % Wheel radius estimates
r_R = (n_ticks*d_R)/(2*pi*delta_tick_R)


% Run controller_l_w to calculate the distance between the wheels
% The average left sensor data when the robot is aligned with the obstacle
minValue_left = min(sensor_record_left); 
% The average right sensor data when the robot is aligned with the obstacle
minValue_right = min(sensor_record_right);
L_w = minValue_right-minValue_left % Distance between the wheels estimates

