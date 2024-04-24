%% Visualize3D
% Uses trajectory data in DATA to create a 3D visualization on a team of
% differential drive wheeled robots.
%%
% This script uses the trajectory data in DATA to create a 3D visualization
% of the trajectories on a team of differential drive wheeled robots. The
% data in DATA should be a N-by-M-by-3 matrix with the first two rows
% corresponding to the robot's position on the plane and the third row
% corresponding to the robot's heading. The second dimension corresponds to
% the state information at each time step in the M-dimensional vector TIME.
% The third dimension corresponds to the number of robots executing the
% trajectories.

%%
% Determine the number of robots and length of simulation from DATA.
load('C:\Users\yancy_000\Documents\Gatech\GRITS Lab\Braiding Videos\Braid Track Loop\Data\PerfectTheta_MediumM_Smalldt_Run');

%Generating a DATA matrix for now.
% DATA = [linspace(0,10,100);linspace(0,10,100);ones(1,100).*pi/4];
DATA(:,:,1) = X;
DATA(:,:,2) = Y;
[N,M,~] = size(DATA);
for i = 1:M
    for j = 1:N
        DATA(j,i,3) = atan2(HeadingTrack(2,j,i),HeadingTrack(1,j,i));
    end
end
Time = t;
%% World Definition
% Now we setup the world environment where the robots will be executing the
% trajectories. We begin by creating the main figure for the animation and
% by generating a grid of the floor.
close all hidden
AnimationWindow = figure('Name','Khepera 3D Animation','NumberTitle','off','Renderer','OpenGL','Color','c');
% GroundPlane = patch([-10,-10,15,15],[-10,15,15,-10],[0,0,0,0],[1 235/255 205/255]);
cameratoolbar
hold on


OuterBox = 2*[3 1.5];
InnerGap = 2*0.8;

% daspect([1 1 1]);
% pbaspect([1 1 1]);
% (set(gca,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1],'cameraviewanglemode','manual','position',[0 0 1 1])
camproj perspective
% axis vis3d
camva(25)
camzoom(0.75)
% camroll(0)

hlight = camlight('headlight'); 
% set(GroundPlane,'AmbientStrength',.75,...
%       'SpecularStrength',0.25,...
%       'DiffuseStrength',0.25);
% set(hcone,'SpecularStrength',1);
% set(gcf,'Color','k')

lighting gouraud

grayIntensity = 0.3;


% GrassH = patch([0 0 OuterBox(1) OuterBox(1) ],[0 OuterBox(2) OuterBox(2) 0 ],0.001.*ones(1,4),'g','EdgeColor','y','facecolor','g');

numTrap = 6;
x = 0:1/numTrap:1;
y = 0:1;
LeftOuterLoop = [OuterBox(2)/2.*cos(-pi.*x-pi/2)+OuterBox(2)/2;OuterBox(2)/2.*sin(-pi.*x-pi/2)+OuterBox(2)/2];
TopOuterLine = [OuterBox(2)/2*(1-y)+(OuterBox(1)-OuterBox(2)/2)*y;OuterBox(2)*ones(size(y))];
RightOuterLoop = [OuterBox(2)/2.*cos(-pi.*x+pi/2)+(OuterBox(1)-OuterBox(2)/2);OuterBox(2)/2.*sin(-pi.*x+pi/2)+(OuterBox(2)/2)];
BottomOuterLine = [OuterBox(2)/2*y+(OuterBox(1)-OuterBox(2)/2)*(1-y);OuterBox(2)*zeros(size(y))];

OuterTrack = [LeftOuterLoop,TopOuterLine([]),RightOuterLoop,BottomOuterLine(:,end)];
OuterTrack = OuterTrack(:,end:-1:1);

LeftInnerLoop = [(InnerGap)/2.*cos(-pi.*x-pi/2)+OuterBox(2)/2;(InnerGap)/2.*sin(-pi.*x-pi/2)+OuterBox(2)/2];
TopInnerLine = [OuterBox(2)/2*(1-y)+(OuterBox(1)-OuterBox(2)/2)*y;(OuterBox(2)/2+InnerGap/2)*ones(size(y))];
RightInnerLoop = [(InnerGap)/2.*cos(-pi.*x+pi/2)+OuterBox(1)-OuterBox(2)/2;(InnerGap)/2.*sin(-pi.*x+pi/2)+OuterBox(2)/2];
BottomInnerLine = [OuterBox(2)/2*y+(OuterBox(1)-OuterBox(2)/2)*(1-y);(OuterBox(2)/2-InnerGap/2)*ones(size(y))];

InnerTrack = [LeftInnerLoop,TopInnerLine([]),RightInnerLoop,BottomInnerLine(:,end)];
InnerTrack = InnerTrack(:,end:-1:1);
% plot(OuterTrack(1,:),OuterTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1],'Marker','none');
% plot(InnerTrack(1,:),InnerTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1],'Marker','none');
% OutTrackH = patch(OuterTrack(1,:),OuterTrack(2,:),ones(size(OuterTrack(1,:)))*0.001,[244 164 96]./255,'EdgeColor',[.3 .3 .3],'FaceColor',[244 164 96]./255);
% InTrackH = patch(InnerTrack(1,:),InnerTrack(2,:),ones(size(InnerTrack(1,:)))*0.003,'g','EdgeColor',[.3 .3 .3],'FaceColor','g');

GroundPlaneVertices = [[-10,-10,15,15]',[-10,15,15,-10]',[0,0,0,0]'; % Ground plane vertices (4)
                       [0 0 OuterBox(1) OuterBox(1) ]',[0 OuterBox(2) OuterBox(2) 0 ]',[0,0,0,0]'; % Grassy Area vertices (4)
                       [OuterTrack(:,1:end-1)',zeros(size(OuterTrack,2)-1,1)]; % Outer track loop vertices (15)
                       [InnerTrack(:,1:end-1)',zeros(size(InnerTrack,2)-1,1)]]; % Inner track loop vertices (15)
% GroundPlaneFaces = [ [1:4,1,5,8:-1:5,ones(1,22)*NaN];
%                      [9,5:8,9:23,ones(1,12)*NaN];
%                      [10:23,9,38:-1:24,38,9];
%                      [24:38,ones(1,17)*NaN]];
Triangulation = delaunay(GroundPlaneVertices(:,1),GroundPlaneVertices(:,2));

% GroundPlaneColor = [1 0.9216 0.8039;
%                     0.4863 0.9882 0;
%                     0.9569 0.6431 0.3765;
%                     0.4863 0.9882 0];

SndCol = [0.4863 0.9882 0];%[1 0.9216 0.8039];
TrkCol = [0.9569 0.6431 0.3765];
GrsCol = [0.4863 0.9882 0];
GroundPlaneColor = ...% There are 66 faces in the triangulation
    [GrsCol;TrkCol;GrsCol;TrkCol;TrkCol;GrsCol;
     TrkCol;TrkCol;TrkCol;TrkCol;GrsCol;SndCol;
     GrsCol;GrsCol;SndCol;GrsCol;SndCol;SndCol;
     GrsCol;GrsCol;TrkCol;GrsCol;TrkCol;GrsCol;
     TrkCol;GrsCol;TrkCol;SndCol;SndCol;SndCol;
     SndCol;TrkCol;GrsCol;GrsCol;GrsCol;TrkCol;
     TrkCol;GrsCol;TrkCol;GrsCol;TrkCol;GrsCol;
     TrkCol;TrkCol;TrkCol;TrkCol;SndCol;TrkCol;
     SndCol;GrsCol;GrsCol;GrsCol;SndCol;TrkCol;
     TrkCol;TrkCol;GrsCol;TrkCol;TrkCol;TrkCol;
     SndCol;TrkCol;GrsCol;GrsCol;SndCol;SndCol];
                    
%                     ones(12,1)*[0.4863 0.9882 0];
%                     ones(28,1)*[0.9569 0.6431 0.3765];
%                     ones(12,1)*[0.4863 0.9882 0]];


GroundPlaneH = trisurf(Triangulation, GroundPlaneVertices(:,1), GroundPlaneVertices(:,2), GroundPlaneVertices(:,3),...
        'FaceVertexCData',GroundPlaneColor(:,:), 'FaceColor','Flat','EdgeColor','None','FaceLighting','gouraud');

% GroundPlaneH = patch('Vertices', GroundPlaneVertices, 'Faces', GroundPlaneFaces(:,:),...
%     'FaceVertexCData',GroundPlaneColor(:,:),'FaceColor','Flat','EdgeColor','none','FaceLighting','gouraud');


% GroundPlaneH = patch('Vertices', GroundPlaneVertices, 'Faces', GroundPlaneFaces(:,:),...
%     'FaceVertexCData',GroundPlaneColor(:,:),'FaceColor','Flat','EdgeColor','none','FaceLighting','gouraud');
         
set(gca,'DataAspectRatio',[1 1 1],'PlotBoxAspectRatio',[1 1 1],...
    'ActivePositionProperty','Position','position',[0 0 1 1])

% set(OutTrackH,'AmbientStrength',.75,...
%       'SpecularStrength',0.25,...
%       'DiffuseStrength',0.25);
% set(InTrackH,'AmbientStrength',.75,...
%       'SpecularStrength',0.25,...
%       'DiffuseStrength',0.25);
% set(GrassH,'AmbientStrength',.75,...
%       'SpecularStrength',0.25,...
%       'DiffuseStrength',0.25);
set(GroundPlaneH,'AmbientStrength',.75,...
      'SpecularStrength',0.1,...
      'DiffuseStrength',0.1);
campos([[6.25 2.5]/2, 12])
camtarget([OuterBox/2,0])
axis manual
axis off
%%
% camcampos([0 0 10])
% cameratoolbar
% axis off
%% Robot 3D Model
% We begin by generating the 3D model for the robot's body.

[RobotFaces,RobotVertices,RobotColor] = RobotBody;
figure(gcf)
Robot(N) = 0;
BraidLines(N) = 0;
for i =1:N
    Robot(i) = patch('Vertices', RobotVertices,'Faces',RobotFaces,'FaceColor','Flat','FaceVertexCData',RobotColor,'EdgeColor','None','FaceLighting','gouraud');
    % set(gca,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1],'cameraviewanglemode','manual')
    % camzoom(3)
    % axis off
    % xlabel('x')
    % ylabel('y')
    % zlabel('z')
    % light('Position',[1 0 5],'Style','infinite');
    % view(3)
    ScaleRobotBody(Robot(i),0.07/2)
    
    MoveRobot(Robot(i),DATA(i,1,1),DATA(i,1,2),DATA(i,1,3))

    BraidLines(i) = patchline(BraidPoint_Track_X(BraidAgents==i),...
         BraidPoint_Track_Y(BraidAgents==i),'EdgeColor',c{i},'linewidth',12,'EdgeAlpha',0.4);

end
%%
camzoom(0.75)


camlookat(Robot)
% campos([DATA(3,1,1),DATA(3,1,2),10]);%
campos([[6.25 2.5]/2, 1/3])
camlight(hlight,'headlight')
drawnow;
pause(Time(2)-Time(1))
for i = 2:10:M
    for j =1:N
        MoveRobot(Robot(j),DATA(j,i,1),DATA(j,i,2),DATA(j,i,3))
    end
    camlookat(Robot)
    campos([[6.25 2.5]/2, 1/3])
    camlight(hlight,'headlight')
    drawnow;
%     pause(Time(i)-Time(i-1))
end
