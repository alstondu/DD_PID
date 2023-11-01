% Controller for Q4

%Step input for both wheels
u=[1,1]; 

%Record the odometry value of each wheels
left_record = [left_record,y(1)];
right_record = [right_record,y(2)];

