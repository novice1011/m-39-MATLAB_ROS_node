%% Clear 
clc; clear all;

%% global variable
pose_sub_x = 0;
pose_sub_y = 0;
%% pub init
vel_pub = rospublisher('/cmd_vel','geometry_msgs/Twist');
pause(2); % Wait to ensure publisher is registered
vel_msg = rosmessage(vel_pub); % pub message

%% sub init
pose_sub = rossubscriber('/odom');
target_1 = [1 0];
target_2 = [0 0];
target=target_1;
pos_case = 1;

%% while loop
ts = 10;
while true
    pose_msg = receive(pose_sub,1);
    pose_sub_x = pose_msg.Pose.Pose.Position.X;
    pose_sub_y = pose_msg.Pose.Pose.Position.Y;
    quat = [pose_msg.Pose.Pose.Orientation.X ...
            pose_msg.Pose.Pose.Orientation.Y ...
            pose_msg.Pose.Pose.Orientation.Z ...
            pose_msg.Pose.Pose.Orientation.W];
    rpy = quat2eul(quat);
    yaw = rpy(3);
    
    x_d = target(1)-pose_sub_x;
    y_d = target(2)-pose_sub_y;
    
    ang = atan2(y_d, x_d);
    dist = sqrt(y_d^2 + x_d^2);
    
    disp([dist abs(ang - yaw)])
    
    if abs(ang - yaw) > (pi/10)
        vel_msg.Linear.X = 0;
        vel_msg.Angular.Z = 0.15*(2*pi)/ts; % 0.25 circle per 10 sec
        send(vel_pub,vel_msg); %send/pub the message
    else
        vel_msg.Linear.X = 1/ts;
        vel_msg.Angular.Z = 0;
        send(vel_pub,vel_msg); %send/pub the message
        if dist < 0.1 && pos_case == 1
            target = target_2;
            pos_case = 2;
        elseif dist < 0.1 && pos_case == 2
            target = target_1;
            pos_case = 1;
        else
            % nothing
        end
    end
end
