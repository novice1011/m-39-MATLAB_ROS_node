# ROS and MATLAB integration  

"Moving turtlebot using MATLAB, ROS toolbox, and GAZEBO simulation"  

For better look, please visit [HackMD.io](https://hackmd.io/@libernormous/matlab_ros_turtlebot)  

by:  
[Liber Normous](https://hackmd.io/@libernormous)  

---

# Requirements  

1. MATLAB 2021a  
2. ROS Toolbox  
3. ROS1  
4. Turtlebot3 Simulation  

----

# Tutorial from MATLAB  

Run turtlebot gazebo empty world simulation.  
```bash=
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Then run this code section by section.  

 
```c=
%% Clear 
clc; clear all;

%% start ROS master in MATLAB
% to connect outside ros master, use; rosinit('http://192.168.1.1:12000')
rosinit

%% RUn GAZEBO then List nodes
rosnode list

%%  Use rostopic list
rostopic list

%%  Use rostopic info <topicname> to get specific information about a specific topic
rostopic info /cmd_vel   

%% List service
rosservice list

%% service info
rosservice info /imu_service

%% shutdown ROS
% rosshutdown

%% subscribing
laser = rossubscriber('/scan');

%% try using data to visualize laser
angle = laser.LatestMessage.AngleMin:laser.LatestMessage.AngleIncrement:laser.LatestMessage.AngleMax;
x = laser.LatestMessage.Ranges.*cos(angle');
y = laser.LatestMessage.Ranges.*sin(angle');
plot(x, y, '.'); axis square;

%% asking/waiting data from subscribes topic
scandata = receive(laser,10);

figure(2)
plot(scandata,'MaximumRange',2) %mistery


%% subscribing using callback
laser = rossubscriber('/scan', @callback_func);

%% Create a publisher that sends ROS string messages to the /chatter topic (see Work with Basic ROS Messages).

chatterpub = rospublisher('/from_zee','std_msgs/String');
chattersub = rossubscriber('/from_zee', @callback_func);
pause(2); % Wait to ensure publisher is registered

%% Create and populate a ROS message to send to the /chatter topic.
chattermsg = rosmessage(chatterpub);
chattermsg.Data = 'hello world';
send(chatterpub,chattermsg);

%% Functions
function callback_func(~,callback_ret)
%exampleHelperROSEmptyCallback Callback function used by a ROS service server
%   exampleHelperROSEmptyCallback(~,~,RESP) returns no arguments. it simply
%   displays a message indicating that it has been called.
%
%   See also ROSServicesExample, exampleHelperROSCreateSampleNetwork.

%   Copyright 2014-2015 The MathWorks, Inc.

disp(callback_ret)
end

```

---

# Moving the robot back and forth  

This code will move the robot between 2 points. after reaching a point, the robot will turn into another point.  
The point reference is `/odom` reading.  That means the origin is the starting point.  

This is the flow chart of the code.  
![](https://i.imgur.com/LkCZCBh.png)  
This is the graph of the program.  
![](https://i.imgur.com/bEq28hs.png)  

Run turtlebot gazebo empty world simulation.  
```bash=
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Then run this code.  
```c=
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Move back and fort using stupid controller%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

```

Result:  
![](https://i.imgur.com/85dG3qo.png)  



---

# References

[1] [Exchange Data with ROS Publishers and Subscribers](https://www.mathworks.com/help/ros/ug/exchange-data-with-ros-publishers-and-subscribers.html)  
[2] [quat2eul](https://www.mathworks.com/help/robotics/ref/quat2eul.html)  
[3] [How to Move a Robot to a Certain Point Using Twist](https://www.youtube.com/watch?v=eJ4QPrYqMlw)   
[4]  
