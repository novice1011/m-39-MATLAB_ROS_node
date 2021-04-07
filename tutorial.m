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

% disp('A service client is calling');
% disp(sum(callback_ret.ranges))
disp(callback_ret)
end

