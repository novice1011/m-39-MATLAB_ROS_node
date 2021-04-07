%% Functions
function pose_callback_func(~,callback_ret)
%exampleHelperROSEmptyCallback Callback function used by a ROS service server
%   exampleHelperROSEmptyCallback(~,~,RESP) returns no arguments. it simply
%   displays a message indicating that it has been called.
%
%   See also ROSServicesExample, exampleHelperROSCreateSampleNetwork.

%   Copyright 2014-2015 The MathWorks, Inc.

% disp('A service client is calling');
% disp(sum(callback_ret.ranges))
% disp(callback_ret.Pose.Pose.Position)
global pose_sub_x;
global pose_sub_y;
pose_sub_x = callback_ret.Pose.Pose.Position.X;
pose_sub_y = callback_ret.Pose.Pose.Position.Y;
% disp([pose_sub_x pose_sub_y])
end