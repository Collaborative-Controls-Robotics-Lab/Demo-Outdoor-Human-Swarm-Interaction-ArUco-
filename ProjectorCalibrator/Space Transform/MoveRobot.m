function MoveRobot( RobotBodyHandle, X, Y, Theta )
%MoveRobot Moves the base of the robot and changes its orientation
%   Moves the base of the robot passed through its handle RobotBodyHandle
%   to the point (X,Y) and changes its orientation to so that the front of
%   the robot is at an angle Theta from the X-axis of the world frame.

%%
% First, extract the (X,Y,Z) data of the robot body from its handle.
RobotData = get(RobotBodyHandle,'Vertices');

%%
% Translate the robot back to the origin in order to rotate it. The first
% point correspond to the center of the base. We'll subtract it from the
% data to shift the robot's base to the origin.
% 
% The 3007th point is directly at the left of the robot. We'll use it to
% determine the robot's current heading by finding the angle from it to the
% base and moving 90 degrees counter clockwise.
BasePosition = RobotData(1,:);

RobotsSide = RobotData(3007,:);

RobotsHeading = ...
   atan2(BasePosition(2)-RobotsSide(2),BasePosition(1)-RobotsSide(1))+pi/2;

% RobotData = RobotData-repmat([BasePosition(1:2),0],size(RobotData,1),1);
RobotData = bsxfun(@minus,RobotData,[BasePosition(1:2),0]);
% RobotYData = RobotYData-BasePosition(2);
% RobotZData = RobotZData-BasePosition(3);

%%
% Change the robot's heading to the desired angle Theta and shift its base
% to its new location (X,Y).

% keyboard
% RotMatZ = @(x) [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1];
% RobotData = (RotMatZ(Theta)*RotMatZ(-RobotsHeading)*RobotData')';
RobotData = ([cos(Theta) -sin(Theta) 0; sin(Theta) cos(Theta) 0; 0 0 1]*[cos(-RobotsHeading) -sin(-RobotsHeading) 0; sin(-RobotsHeading) cos(-RobotsHeading) 0; 0 0 1]*RobotData')';

% RobotData = RobotData + repmat([X,Y,0],size(RobotData,1),1);
RobotData = bsxfun(@plus,RobotData,[X,Y,0]);

set(RobotBodyHandle,'Vertices',RobotData);
end

