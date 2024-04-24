function [ RobotFaces, RobotVertices, RobotColor ] = RobotBody
%RobotBody Returns the patch info needed to draw the 3D model of a robot
%   Returns the vertices, faces and color information required to draw the
%   3D model of a robot's body by using the Patch function.
% 
% 
% See also PATCH.

%%
HandleVisibility = 'on';
RobotBodyFig = figure('Visible',HandleVisibility);
hold off
[WheelX,WheelY,WheelZ] = cylinder([0,0.5*ones(1,100),0],100);%patch(surf2patch
WheelR = surf(WheelX(:,round(end/2):end)-0.5,WheelY(:,round(end/2):end)-0.25,WheelZ(:,round(end/2):end)*0.05+0.25-0.05/2,'FaceColor',[0.1 0.1 0.1],'EdgeColor','none','FaceLighting','gouraud','Visible',HandleVisibility);
hold on
WheelL = surf(WheelX(:,round(end/2):end)-0.5,WheelY(:,round(end/2):end)-0.25,WheelZ(:,round(end/2):end)*0.05+0.25-0.05/2,'FaceColor',[0.1 0.1 0.1],'EdgeColor','none','FaceLighting','gouraud','Visible',HandleVisibility);
% WheelTopR = trisurf(delaunay(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25),...
%     WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25,WheelZ(1,round(end/2):end)+0.25+0.05/2,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceLighting','gouraud');%%NextPlot','Add
% WheelBotR = trisurf(delaunay(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25),...
%     WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25,WheelZ(1,round(end/2):end)+0.25-0.05/2,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceLighting','gouraud');
% WheelTopL = trisurf(delaunay(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25),...
%     WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25,WheelZ(1,round(end/2):end)+0.25+0.05/2,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceLighting','gouraud');
% WheelBotL = trisurf(delaunay(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25),...
%     WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25,WheelZ(1,round(end/2):end)+0.25-0.05/2,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceLighting','gouraud');
% WheelTopR = patch(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25,WheelZ(1,round(end/2):end)+0.25+0.05/2,[0.7 0.7 0.7],'EdgeColor','none','FaceLighting','gouraud');%%NextPlot','Add
% WheelBotR = patch(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25,WheelZ(1,round(end/2):end)+0.25-0.05/2,[0.7 0.7 0.7],'EdgeColor','none','FaceLighting','gouraud');
% WheelTopL = patch(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25,WheelZ(1,round(end/2):end)+0.25+0.05/2,[0.7 0.7 0.7],'EdgeColor','none','FaceLighting','gouraud');
% WheelBotL = patch(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)-0.25,WheelZ(1,round(end/2):end)+0.25-0.05/2,[0.7 0.7 0.7],'EdgeColor','none','FaceLighting','gouraud');
rotate(WheelR,[1 0 0],90,[-0.5,-0.25,0.5])
% rotate(WheelTopR,[1 0 0],90,[-0.5,-0.25,0.5])
% rotate(WheelBotR,[1 0 0],90,[-0.5,-0.25,0.5])
rotate(WheelL,[1 0 0],90,[-0.5,-0.25,0.5])
% rotate(WheelTopL,[1 0 0],90,[-0.5,-0.25,0.5])
% rotate(WheelBotL,[1 0 0],90,[-0.5,-0.25,0.5])
set(WheelR,'YData',get(WheelR,'YData')-0.75);
% set(WheelTopR,'YData',get(WheelTopR,'YData')-0.75);
% set(WheelBotR,'YData',get(WheelBotR,'YData')-0.75);
set(WheelL,'YData',get(WheelL,'YData')+0.75);
% set(WheelTopL,'YData',get(WheelTopL,'YData')+0.75);
% set(WheelBotL,'YData',get(WheelBotL,'YData')+0.75);
set(gca,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1],'cameraviewanglemode','manual')

%
RobotBodyContour = [0,sin(0:pi/2/49:pi/2)*0.1+0.9,ones(1,100*5),sin(pi/2:pi/4/199:3*pi/4),0];

[RobotBodyRoundedX,RobotBodyRoundedY,RobotBodyRoundedZ] ...
    = cylinder(RobotBodyContour,99); 

RobotBodyRectangular = [zeros(1,size(RobotBodyRoundedX,1)),zeros(1,size(RobotBodyRoundedX,1));
                        RobotBodyRoundedX(:,1)',RobotBodyRoundedX(end:-1:1,end/2-1)';
                        linspace(0,1,size(RobotBodyRoundedX,1)),linspace(1,0,size(RobotBodyRoundedX,1))];
% clf

% plot3([RobotBodyRectangular(1,1:end/2);RobotBodyRectangular(1,end/2+1:end);RobotBodyRectangular(1,1:end/2)-1;RobotBodyRectangular(1,end/2+1:end)-1;-RobotBodyVerticesX(:,25:76)';-RobotBodyVerticesX(:,[23:25,76:78])'-1+max(max(RobotBodyVerticesX(:,[23:25,76:78])))]',...
%       [repmat([RobotBodyRectangular(2,1:end/2);RobotBodyRectangular(2,end/2+1:end)],2,1);RobotBodyVerticesY(:,25:76)';RobotBodyVerticesY(:,[23:25,76:78])']',...
%       [repmat([RobotBodyRectangular(3,1:end/2);RobotBodyRectangular(3,end/2+1:end)],2,1);RobotBodyVerticesZ(:,25:76)';RobotBodyVerticesZ(:,[23:25,76:78])']');

RobotBodyX = [RobotBodyRectangular(1,1:end/2);RobotBodyRectangular(1,end/2+1:end);RobotBodyRectangular(1,1:end/2)-1;RobotBodyRectangular(1,end/2+1:end)-1;-RobotBodyRoundedX(:,25:76)';-RobotBodyRoundedX(:,[23:25,76:78])'-1]';
RobotBodyY = [repmat([RobotBodyRectangular(2,1:end/2);RobotBodyRectangular(2,end/2+1:end)],2,1);RobotBodyRoundedY(:,25:76)';RobotBodyRoundedY(:,[23:25,76:78])']';
RobotBodyZ = [repmat([RobotBodyRectangular(3,1:end/2);RobotBodyRectangular(3,end/2+1:end)],2,1);RobotBodyRoundedZ(:,25:76)';RobotBodyRoundedZ(:,[23:25,76:78])']'*0.8+0.2;
Triangulation = convhull(RobotBodyX,RobotBodyY,RobotBodyZ, 'simplify',true);

% RobotBodyX(K);
% RobotBodyY(K);
% RobotBodyZ(K);
%
RobotBodyHull = trisurf(Triangulation,RobotBodyX,RobotBodyY,RobotBodyZ,'EdgeColor','none','FaceColor',[0.7 0.7 0.7],'FaceLighting','gouraud','Visible',HandleVisibility);


[FrontPivotX,FrontPivotY,FrontPivotZ] = cylinder([0,linspace(0.1,0.3,100)],100);
FrontPivot = surf(FrontPivotX+0.5,FrontPivotY,FrontPivotZ*0.3,'FaceColor',[0.3 0.3 0.3],'EdgeColor','none','FaceLighting','gouraud','Visible',HandleVisibility);
% axis equal

[WheelRFaces,WheelRVertices,~] = surf2patch(WheelR,'triangles');
[WheelLFaces,WheelLVertices,~] = surf2patch(WheelL,'triangles');
[FrontPivotFaces,FrontPivotVertices,~] = surf2patch(FrontPivot,'triangles');

% [RobotBodyFaces,RobotBodyVertices,RobotBodyColor] = surf2patch(RobotBodyHull,'triangles');
RobotBodyFaces = get(RobotBodyHull,'Faces');
RobotBodyVertices = get(RobotBodyHull,'Vertices');
% RobotBodyColor = get(RobotBodyHull,'FaceVertexCData');

RobotBodyColor = 0.7.*ones(size(RobotBodyFaces,1),3);
Ind1 = find(ismember(RobotBodyVertices,[0 0 1],'rows'));
[ii1, ~] = ind2sub(size(RobotBodyFaces),find(RobotBodyFaces(:)==Ind1(1)));
% RobotBodyColor(ii1,:) = RobotBodyColor(ii1,:)./0.7*0.1;
Ind2 = find(ismember(RobotBodyVertices,[-1 0 1],'rows'));
[ii2, ~] = ind2sub(size(RobotBodyFaces),find(RobotBodyFaces(:)==Ind2(1)));
ii = unique([ii1;ii2]);
RobotBodyColor(ii,:) = RobotBodyColor(ii,:)./0.7*0.1;
set(RobotBodyHull,'FaceVertexCData',RobotBodyColor,'FaceColor','Flat')

WheelRColor = 0.1.*ones(size(WheelRFaces,1),3);
WheelLColor = 0.1.*ones(size(WheelLFaces,1),3);
FrontPivotColor = 0.1.*ones(size(FrontPivotFaces,1),3);

% WheelTopRFaces = get(WheelTopR,'Faces');
% WheelTopRVertices = get(WheelTopR,'Vertices');
% WheelTopRColor = get(WheelTopR,'FaceVertexCData');
% 
% WheelTopLFaces = get(WheelTopL,'Faces');
% WheelTopLVertices = get(WheelTopL,'Vertices');
% WheelTopLColor = get(WheelTopL,'FaceVertexCData');
% 
% WheelBotRFaces = get(WheelBotR,'Faces');
% WheelBotRVertices = get(WheelBotR,'Vertices');
% WheelBotRColor = get(WheelBotR,'FaceVertexCData');
% 
% WheelBotLFaces = get(WheelBotL,'Faces');
% WheelBotLVertices = get(WheelBotL,'Vertices');
% WheelBotLColor = get(WheelBotL,'FaceVertexCData');

% RobotVertices = [RobotBodyVertices;WheelTopRVertices;WheelRimRVertices;WheelBotRVertices;WheelTopLVertices;WheelRimLVertices;WheelBotLVertices;FrontPivotVertices];
RobotVertices = [RobotBodyVertices;WheelRVertices;WheelLVertices;FrontPivotVertices];

% RobotFaces = [RobotBodyFaces;size(RobotBodyFaces,1)+WheelTopRFaces];
% RobotFaces = [RobotFaces;size(RobotFaces,1)+WheelRimRFaces];
% RobotFaces = [RobotFaces;size(RobotFaces,1)+WheelBotRFaces];
% RobotFaces = [RobotFaces;size(RobotFaces,1)+WheelTopLFaces];
% RobotFaces = [RobotFaces;size(RobotFaces,1)+WheelBotLFaces];

% RobotFaces = [RobotBodyFaces;size(RobotBodyFaces,1)+WheelRFaces];
% RobotFaces = [RobotFaces;size(RobotFaces,1)+WheelLFaces];
% RobotFaces = [RobotFaces;size(RobotFaces,1)+FrontPivotFaces];
RobotFaces = [RobotBodyFaces;size(RobotBodyVertices,1)+WheelRFaces];
RobotFaces = [RobotFaces;size([RobotBodyVertices;WheelRVertices],1)+WheelLFaces];
RobotFaces = [RobotFaces;size([RobotBodyVertices;WheelRVertices;WheelLVertices],1)+FrontPivotFaces];

% RobotColor = [RobotBodyColor;WheelTopRColor;WheelRimRColor;WheelBotRColor;WheelTopLColor;WheelRimLColor;WheelBotLColor;FrontPivotColor];
RobotColor = [RobotBodyColor;WheelRColor;WheelLColor;FrontPivotColor];



set(gca,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1],'cameraviewanglemode','manual')
camzoom(3)
axis off
xlabel('x')
ylabel('y')
zlabel('z')
light('Position',[1 0 5],'Style','infinite');
view(3)
%%
close(RobotBodyFig)
%{
%%
hold on
WheelRimL = surf(WheelX(:,round(end/2):end)-0.5,WheelY(:,round(end/2):end)+0.75,WheelZ(:,round(end/2):end)*0.05+0.25-0.05/2,'FaceColor',[0.1 0.1 0.1],'EdgeColor','none');
WheelTopL = patch(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)+0.75,WheelZ(1,round(end/2):end)+0.25+0.05/2,[0.7 0.7 0.7],'EdgeColor','none');%%NextPlot','Add
WheelBotL = patch(WheelX(1,round(end/2):end)-0.5,WheelY(1,round(end/2):end)+0.75,WheelZ(1,round(end/2):end)+0.25-0.05/2,[0.7 0.7 0.7],'EdgeColor','none');%%NextPlot','Add
%%
rotate(WheelRimL,[1 0 0],90)
rotate(WheelTopL,[1 0 0],90)
rotate(WheelBotL,[1 0 0],90)
% rotate(Wheel,[0 0 1],90)
%}
end

