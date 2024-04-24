function ScaleRobotBody( RobotBodyHandle, Size )
%ScaleRobotBody Scale the size of a robot passed through its handle to Size
%   Given a robot handle RobotBodyHandle, this functions makes it so that
%   the robot body fits on a box of dimensions
%   [-Size,Size]-by-[-Size,Size]-by-[0,Size]

%%
% Translate the robot back to the origin in order to scale it.
RobotXData = get(RobotBodyHandle,'XData');
RobotYData = get(RobotBodyHandle,'YData');
RobotZData = get(RobotBodyHandle,'ZData');

%%
% The first point correspond to the center of the base. We'll subtract it
% from the data to shift the robot's centroid to the origin.
BasePosition = [RobotXData(1),RobotYData(1),RobotZData(1)];

RobotXData = RobotXData-BasePosition(1);
RobotYData = RobotYData-BasePosition(2);
RobotZData = RobotZData-BasePosition(3);

%%
% Normalize the data so that it fits in a [-1,1]-by-[-1,1]-by-[0,1]
RobotXData = 2.*RobotXData./(max(RobotXData(:))-min(RobotXData(:)));
RobotYData = 2.*RobotYData./(max(RobotYData(:))-min(RobotYData(:)));
RobotZData = 2.*RobotZData./((max(RobotZData(:))-min(RobotZData(:))));

%%
% Multiplying by the scaling factor and raise the base back to the ground
RobotXData = RobotXData.*Size;
RobotYData = RobotYData.*Size;
RobotZData = RobotZData.*Size/2+Size/4;

%%
% Return the base to the original point and update the handle
RobotXData = RobotXData+BasePosition(1);
RobotYData = RobotYData+BasePosition(2);
% RobotZData = RobotZData+BasePosition(3);

set(RobotBodyHandle,'XData',RobotXData,'YData',RobotYData,'ZData',RobotZData);

end

