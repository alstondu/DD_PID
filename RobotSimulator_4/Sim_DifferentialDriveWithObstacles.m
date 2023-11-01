% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 25;
dt          = 0.02;
TIME_SCALE  = 0.01; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)


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
init_Q4;
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
    controller_Q4;
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
%Record the last 100 samples of odometry change for each wheel
delta_odo_left = left_record(end) - left_record(end-100);
delta_odo_right = right_record(end) - right_record(end-100);

%Calculate the distance of the wheels from the odometry change
delta_d_left = (delta_odo_left / 64) * (2 * pi * r);
delta_d_right = (delta_odo_right / 64) * (2 * pi * r);

%Calculate the average distance of the two wheels
delta_d_ave = (delta_d_left + delta_d_right)/2;

%Calculate the average speed
s_m = delta_d_ave/(100*dt);

%Based on the step input, transferfunction provided and the final value
%theorem, when s = 0, Ks = v_ave
Ks = s_m