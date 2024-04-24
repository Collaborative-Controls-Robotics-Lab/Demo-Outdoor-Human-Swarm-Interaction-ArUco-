%% STRAIGHTENEDPATH Sets up the space to perform braids on a track.
clear all, close all, clc
%% Defining Braid Parameters
% Number of Agents
N = 5;

% Number of Braids
M = 20;%2*M_Rectangles+2*numTrap*M_Trapezoids;

% Simulation time
dt = 0.01; % seconds

% Terminal time
tf = 30; % seconds


%%
close all
tic;
TrackPlot = figure('Name','Braid Track','NumberTitle','off','Position',[100 100 850 600]); %[11 46 1580 767]); %[-1279 -123 1280 947]);% [3201 -179 1920 1003]
subplot(4,1,[1 2 3])
grid on
hold on

% Some colors in a cell, e.g. for plotting in loops
grayIntensity = 0.3;
c = {'g','r','m','b','c','k','y'};

% Define the width and height of track in decimeters, convert to meters
OuterBox = 2*[3 1.5];
InnerGap = 2*0.8;
plot([0 0 OuterBox(1) OuterBox(1) 0],[0 OuterBox(2) OuterBox(2) 0 0],'-y','linewidth',3)
axis tight
CurrentAxis = axis;
axis([CurrentAxis(1)-0.025*diff(CurrentAxis(3:4)),...
      CurrentAxis(2)+0.025*diff(CurrentAxis(3:4)),...
      CurrentAxis(3)-0.025*diff(CurrentAxis(3:4)),...
      CurrentAxis(4)+0.025*diff(CurrentAxis(3:4))])
set(gca,'PlotBoxAspectRatio',[1 1 1])
set(gca,'DataAspectRatio',[1 1 1])
title('Track (Real World)','fontsize',14)
xlabel('East (m)','fontsize',14)
ylabel('North (m)','fontsize',14)
axis manual

%Number of Trapezoids
numTrap = 4;
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

plot(OuterTrack(1,:),OuterTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1],'Marker','+');
plot(InnerTrack(1,:),InnerTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1],'Marker','+');

plot(OuterTrack(1,1),OuterTrack(2,1),'lineStyle','none','Color','r','Marker','+');
plot(InnerTrack(1,1),InnerTrack(2,1),'lineStyle','none','Color','r','Marker','+');

% Straighten the path into a long rectangular path.
% First, the average width between the trapezoid's parallel lines. Assuming
% all trapezoids have the same dimensions.
TrapezoidWidth = 0.5*(norm(RightInnerLoop(:,1)-RightInnerLoop(:,2))+norm(RightOuterLoop(:,1)-RightOuterLoop(:,2)));

% Now, straighten the path by appending the straightened-now-rectangles
% "trapezoids" to the right ...
StraigthenedPath_InnerBoundary = [BottomInnerLine(:,end),... % Technical Note: BottomInnerLine is backwards, e.g. pick end point to get first point
    repmat(BottomInnerLine(:,1),1,numTrap+1)+[TrapezoidWidth;0]*(0:numTrap),...
    repmat(BottomInnerLine(:,1)+[TrapezoidWidth;0]*(numTrap)+abs(TopInnerLine(:,1)-TopInnerLine(:,end)),1,numTrap+1)+[TrapezoidWidth;0]*(0:numTrap)];

StraigthenedPath_OuterBoundary = [BottomOuterLine(:,end),...
    repmat(BottomOuterLine(:,1),1,numTrap+1)+[TrapezoidWidth;0]*(0:numTrap),...
    repmat(BottomOuterLine(:,1)+[TrapezoidWidth;0]*(numTrap)+abs(TopOuterLine(:,1)-TopOuterLine(:,end)),1,numTrap+1)+[TrapezoidWidth;0]*(0:numTrap)];

subplot(4,1,4)
% grid on
hold on
plot([StraigthenedPath_InnerBoundary(1,:),StraigthenedPath_OuterBoundary(1,end:-1:1),StraigthenedPath_InnerBoundary(1,1)],...
     [StraigthenedPath_InnerBoundary(2,:),StraigthenedPath_OuterBoundary(2,end:-1:1),StraigthenedPath_InnerBoundary(2,1)],...
     'Color',grayIntensity*[1 1 1],'Linewidth',2,'Marker','+')
plot([StraigthenedPath_InnerBoundary(1,1),StraigthenedPath_InnerBoundary(1,end)],...
     [StraigthenedPath_InnerBoundary(2,1),StraigthenedPath_InnerBoundary(2,end)],...
     'Color','r','LineStyle','none','Marker','+')
plot([StraigthenedPath_OuterBoundary(1,1),StraigthenedPath_OuterBoundary(1,end)],...
     [StraigthenedPath_OuterBoundary(2,1),StraigthenedPath_OuterBoundary(2,end)],...
     'Color','r','LineStyle','none','Marker','+')
axis tight
CurrentAxis2 = axis;
axis([CurrentAxis2(1)-0.1*diff(CurrentAxis2(3:4)),...
      CurrentAxis2(2)+0.1*diff(CurrentAxis2(3:4)),...
      CurrentAxis2(3)-0.1*diff(CurrentAxis2(3:4)),...
      CurrentAxis2(4)+0.1*diff(CurrentAxis2(3:4))])
set(gca,'PlotBoxAspectRatio',[1 1 1])
set(gca,'DataAspectRatio',[1 1 1])
title('Straightened Track (Virtual World)','fontsize',14)
xlabel('Counter Clockwise along Path (m, along Center of Path)','fontsize',14)
ylabel({'From Outer to Inner'; 'Track Boundary (m)'},'fontsize',14)
axis manual

%% Compute the Perspective Transform to Map from Rectangular Plane to Real World

% Compute the decision boundaries in the real plane to determine which
% transform to use.
W = zeros(3,size(InnerTrack,2)-1);
for i = 1:size(InnerTrack,2)-1
   W(:,i) = [InnerTrack(2,i)-OuterTrack(2,i);-(InnerTrack(1,i)-OuterTrack(1,i));det([InnerTrack(:,i),OuterTrack(:,i)])];
end
W = circshift(W,[0 -1]);
% Define the structure of the perspective transforms
T = @(H,x) [eye(2) zeros(2,1)]*(H*[x;1]/(H(3,:)*[x;1]));

% Define the partial derivatives of the transformation
dTdx1 = @(H,x) ...
    [(H(1,1)-H(1,3)*H(3,1)+(H(1,1)*H(3,2)-H(1,2)*H(3,1))*x(2))/(H(3,1:2)*x+1)^2;
     (H(2,1)-H(2,3)*H(3,1)+(H(2,1)*H(3,2)-H(2,2)*H(3,1))*x(2))/(H(3,1:2)*x+1)^2]; 

dTdx2 = @(H,x) ...
    [(H(1,2)-H(1,3)*H(3,2)+(H(1,2)*H(3,1)-H(1,1)*H(3,2))*x(1))/(H(3,1:2)*x+1)^2;
     (H(2,2)-H(2,3)*H(3,2)+(H(2,2)*H(3,1)-H(2,1)*H(3,2))*x(1))/(H(3,1:2)*x+1)^2]; 

% Define the Metric Tensor (Tensor)
MT = @(H,x) [dTdx1(H,x).'*dTdx1(H,x),dTdx1(H,x).'*dTdx2(H,x);
            dTdx1(H,x).'*dTdx2(H,x),dTdx2(H,x).'*dTdx2(H,x)];

% Allocating memory to store the perspective transforms
numTransforms = 2*(numTrap+1);
HH = reshape(repmat(eye(3),1,numTransforms),3,3,[]);

for i = 1:numTransforms
    % Get the corner of the rectangular area: 
    % [InnerBack, OuterBack,OuterFront,InnerFront];
    RectangleCorners = ...
        [StraigthenedPath_InnerBoundary(:,i),...
         StraigthenedPath_OuterBoundary(:,i),...
         StraigthenedPath_OuterBoundary(:,i+1),...
         StraigthenedPath_InnerBoundary(:,i+1)];
    
    TrapezoidCorners = ...
        [InnerTrack(:,i),...
         OuterTrack(:,i),...
         OuterTrack(:,i+1),...
         InnerTrack(:,i+1)];
    
    A = [RectangleCorners(1,1) RectangleCorners(2,1) 1 0 0 0 -TrapezoidCorners(1,1)*RectangleCorners(1,1) -TrapezoidCorners(1,1)*RectangleCorners(2,1);
         0 0 0 RectangleCorners(1,1) RectangleCorners(2,1) 1 -TrapezoidCorners(2,1)*RectangleCorners(1,1) -TrapezoidCorners(2,1)*RectangleCorners(2,1);
         RectangleCorners(1,2) RectangleCorners(2,2) 1 0 0 0 -TrapezoidCorners(1,2)*RectangleCorners(1,2) -TrapezoidCorners(1,2)*RectangleCorners(2,2);
         0 0 0 RectangleCorners(1,2) RectangleCorners(2,2) 1 -TrapezoidCorners(2,2)*RectangleCorners(1,2) -TrapezoidCorners(2,2)*RectangleCorners(2,2);
         RectangleCorners(1,3) RectangleCorners(2,3) 1 0 0 0 -TrapezoidCorners(1,3)*RectangleCorners(1,3) -TrapezoidCorners(1,3)*RectangleCorners(2,3);
         0 0 0 RectangleCorners(1,3) RectangleCorners(2,3) 1 -TrapezoidCorners(2,3)*RectangleCorners(1,3) -TrapezoidCorners(2,3)*RectangleCorners(2,3);
         RectangleCorners(1,4) RectangleCorners(2,4) 1 0 0 0 -TrapezoidCorners(1,4)*RectangleCorners(1,4) -TrapezoidCorners(1,4)*RectangleCorners(2,4);
         0 0 0 RectangleCorners(1,4) RectangleCorners(2,4) 1 -TrapezoidCorners(2,4)*RectangleCorners(1,4) -TrapezoidCorners(2,4)*RectangleCorners(2,4)];
    b = [TrapezoidCorners(:,1);TrapezoidCorners(:,2);TrapezoidCorners(:,3);TrapezoidCorners(:,4)]; % Same as "b = RectangleCorners(:);"...

%     keyboard

    % The parameters of the transform in matrix form are given by
    H = reshape([(A\b);1],3,3)';

    % Let's store the transformation parameters
    HH(:,:,i) = H;
end

%% Defining Braid Parameters
% % Number of Agents
% N = 5;
% 
% % Number of Braids
% M = 12;%2*M_Rectangles+2*numTrap*M_Trapezoids;
% M_Trapezoids = 4;
% M_Rectangles = 20;

% Allocating memory for the Braid points
BraidPoints = zeros(2,N,M+1);
BraidPoints_Track = zeros(2,N,M+1);
HH_Braid = reshape(repmat(eye(3),1,(M)*(N-1)),3,3,N-1,[]);

% Find the Braid Points by distributing the M points uniformly along the
% straightened path.
ii = 1;
% jj = 1;
subplot(4,1,[1 2 3])
TrapezoidPlot_Enlarged = plot(zeros(4,1),zeros(4,1),'Marker','o','MarkerSize',10,'color','r');
TrapezoidPlot = plot(zeros(4,1),zeros(4,1),'Marker','o','MarkerSize',10);
subplot(4,1,4)
RectanglePlot_Enlarged = plot(zeros(4,1),zeros(4,1),'Marker','o','MarkerSize',10,'Color','r');
RectanglePlot = plot(zeros(4,1),zeros(4,1),'Marker','o','MarkerSize',10);

% Allocate the transform regions
RectanglePolygons_X = zeros((N-1)*M,5);
RectanglePolygons_Y = zeros((N-1)*M,5);
TrapezoidPolygons_X = zeros((N-1)*M,5);
TrapezoidPolygons_Y = zeros((N-1)*M,5);
RectanglePolygons_X_Enlarged = zeros((N-1)*M,5);
RectanglePolygons_Y_Enlarged = zeros((N-1)*M,5);
TrapezoidPolygons_X_Enlarged = zeros((N-1)*M,5);
TrapezoidPolygons_Y_Enlarged = zeros((N-1)*M,5);
PolygonExpansionFactor = 1.001;
PolygonExpansionFactor_X = 1.1;
PolygonExpansionFactor_Y = 1.2;
% For each braid step:
for i = 1:M+1
    % Set the x coordinate constant,
    % interpolating between begining and end of path.
    BraidPoints(1,:,i) = repmat(...
        StraigthenedPath_InnerBoundary(1,1)*(1-((i-1)/(M)))+StraigthenedPath_InnerBoundary(1,end)*((i-1)/(M)),...
        N,1);
    BraidPoints(2,1,i) = StraigthenedPath_InnerBoundary(2,1);
    BraidPoints(2,end,i) = StraigthenedPath_OuterBoundary(2,1);
    % Interpolate between inner and outer boundaries to obtain
    % uniform spread of agents at each step.
        BraidPoints(2,:,i) = (...
            StraigthenedPath_InnerBoundary(2,1)*(1-((0:N-1)/(N-1)))+StraigthenedPath_OuterBoundary(2,1)*((0:N-1)/(N-1))...
            ).';
    % Using the transforms, plot the braid points in the track. The
    % appropriate transform to use is a function of the horizontal
    % distance in the straightened track.
    if((BraidPoints(1,1,i)>StraigthenedPath_InnerBoundary(1,ii+1)))
        if(ii <= size(StraigthenedPath_InnerBoundary,2))
            if((BraidPoints(1,1,i)>StraigthenedPath_InnerBoundary(1,ii+2)))
                fprintf('Exceeded the first transform range.\n')
                fprintf('Increase number of braid points, decrease number of trapezoids, or rework the code to make it work (you lazy bum...)\n')
                break
            end
        end
        ii = ii + 1;
    end
    % if((BraidPoints(1,1,i+1)>StraigthenedPath_InnerBoundary(1,jj+1)))
    %     if(jj <= size(StraigthenedPath_InnerBoundary,2))
    %         if((BraidPoints(1,1,i+1)>StraigthenedPath_InnerBoundary(1,jj+2)))
    %             fprintf('Exceeded the first transform range.\n')
    %             fprintf('Increase number of braid points, decrease number of trapezoids, or rework the code to make it work (you lazy bum...)\n')
    %             error('Skipped a transform')
    %         end
    %     end
    %     jj = jj + 1;
    % end

    % Compute the braid end points in real world at step i
    BraidPoints_Track(:,1,i) = T(HH(:,:,ii),BraidPoints(:,1,i));
    BraidPoints_Track(:,end,i) = T(HH(:,:,ii),BraidPoints(:,end,i));
    
    BraidPoints_Track(:,:,i) = BraidPoints_Track(:,1,i)*(1-((0:N-1)/(N-1)))+BraidPoints_Track(:,end,i)*((0:N-1)/(N-1));

%     for j = 1:N
%         BraidPoints(:,j,i) = T(inv(HH(:,:,ii)),BraidPoints_Track(:,j,i));
%     end
    % Let's compute a transform for each braid step
    if i>1
        for j = 1:N-1
            % Get the corner of the rectangular area:
            % [InnerBack, OuterBack,OuterFront,InnerFront];
            RectangleCorners = ...
               [BraidPoints(:,j,i-1),...
                BraidPoints(:,j+1,i-1),...
                BraidPoints(:,j+1,i),...
                BraidPoints(:,j,i)];
            
            TrapezoidCorners = ...
               [BraidPoints_Track(:,j,i-1),...
                BraidPoints_Track(:,j+1,i-1),...
                BraidPoints_Track(:,j+1,i),...
                BraidPoints_Track(:,j,i)];
            
            
            
            A = [RectangleCorners(1,1) RectangleCorners(2,1) 1 0 0 0 -TrapezoidCorners(1,1)*RectangleCorners(1,1) -TrapezoidCorners(1,1)*RectangleCorners(2,1);
                0 0 0 RectangleCorners(1,1) RectangleCorners(2,1) 1 -TrapezoidCorners(2,1)*RectangleCorners(1,1) -TrapezoidCorners(2,1)*RectangleCorners(2,1);
                RectangleCorners(1,2) RectangleCorners(2,2) 1 0 0 0 -TrapezoidCorners(1,2)*RectangleCorners(1,2) -TrapezoidCorners(1,2)*RectangleCorners(2,2);
                0 0 0 RectangleCorners(1,2) RectangleCorners(2,2) 1 -TrapezoidCorners(2,2)*RectangleCorners(1,2) -TrapezoidCorners(2,2)*RectangleCorners(2,2);
                RectangleCorners(1,3) RectangleCorners(2,3) 1 0 0 0 -TrapezoidCorners(1,3)*RectangleCorners(1,3) -TrapezoidCorners(1,3)*RectangleCorners(2,3);
                0 0 0 RectangleCorners(1,3) RectangleCorners(2,3) 1 -TrapezoidCorners(2,3)*RectangleCorners(1,3) -TrapezoidCorners(2,3)*RectangleCorners(2,3);
                RectangleCorners(1,4) RectangleCorners(2,4) 1 0 0 0 -TrapezoidCorners(1,4)*RectangleCorners(1,4) -TrapezoidCorners(1,4)*RectangleCorners(2,4);
                0 0 0 RectangleCorners(1,4) RectangleCorners(2,4) 1 -TrapezoidCorners(2,4)*RectangleCorners(1,4) -TrapezoidCorners(2,4)*RectangleCorners(2,4)];
            b = [TrapezoidCorners(:,1);TrapezoidCorners(:,2);TrapezoidCorners(:,3);TrapezoidCorners(:,4)]; % Same as "b = RectangleCorners(:);"...
            
            %     keyboard
            
            % Store the transform regions
            RectanglePolygons_X(j+(N-1)*(i-2),:) = [RectangleCorners(1,:),RectangleCorners(1,1)];
            RectanglePolygons_Y(j+(N-1)*(i-2),:) = [RectangleCorners(2,:),RectangleCorners(2,1)];
            TrapezoidPolygons_X(j+(N-1)*(i-2),:) = [TrapezoidCorners(1,:),TrapezoidCorners(1,1)];
            TrapezoidPolygons_Y(j+(N-1)*(i-2),:) = [TrapezoidCorners(2,:),TrapezoidCorners(2,1)];
            
            mrx=mean(RectangleCorners(1,:));mry=mean(RectangleCorners(2,:)); % x and y are the cordiantes of present polygon
            mtx=mean(TrapezoidCorners(1,:));mty=mean(TrapezoidCorners(2,:)); % x and y are the cordiantes of present polygon
       
            RectanglePolygons_X_Enlarged(j+(N-1)*(i-2),:) = (PolygonExpansionFactor*([RectangleCorners(1,:),RectangleCorners(1,1)]-mrx))+mrx;
            RectanglePolygons_Y_Enlarged(j+(N-1)*(i-2),:) = (PolygonExpansionFactor*([RectangleCorners(2,:),RectangleCorners(2,1)]-mry))+mry;
            TrapezoidPolygons_X_Enlarged(j+(N-1)*(i-2),:) = (PolygonExpansionFactor*([TrapezoidCorners(1,:),TrapezoidCorners(1,1)]-mtx))+mtx;
            TrapezoidPolygons_Y_Enlarged(j+(N-1)*(i-2),:) = (PolygonExpansionFactor*([TrapezoidCorners(2,:),TrapezoidCorners(2,1)]-mty))+mty;
       
            set(TrapezoidPlot_Enlarged,'XData',TrapezoidPolygons_X_Enlarged(j+(N-1)*(i-2),1:end-1),'YData',TrapezoidPolygons_Y_Enlarged(j+(N-1)*(i-2),1:end-1))
            set(RectanglePlot_Enlarged,'XData',RectanglePolygons_X_Enlarged(j+(N-1)*(i-2),1:end-1),'YData',RectanglePolygons_Y_Enlarged(j+(N-1)*(i-2),1:end-1))
            set(TrapezoidPlot,'XData',TrapezoidCorners(1,:),'YData',TrapezoidCorners(2,:))
            set(RectanglePlot,'XData',RectangleCorners(1,:),'YData',RectangleCorners(2,:))
            figure(gcf)
%             pause

            % The parameters of the transform in matrix form are given by
            H = reshape([(A\b);1],3,3)';
            
            % Let's store the transformation parameters
            HH_Braid(:,:,j,i-1) = H;
        end

    end
end

delete(TrapezoidPlot_Enlarged)%,'XData',TrapezoidPolygons_X_Enlarged(j*(i-1),1:end-1),'YData',TrapezoidPolygons_Y_Enlarged(j*(i-1),1:end-1))
delete(RectanglePlot_Enlarged)%,'XData',RectanglePolygons_X_Enlarged(j*(i-1),1:end-1),'YData',RectanglePolygons_Y_Enlarged(j*(i-1),1:end-1))
delete(TrapezoidPlot)%,'XData',TrapezoidCorners(1,:),'YData',TrapezoidCorners(2,:))
delete(RectanglePlot)%,'XData',RectangleCorners(1,:),'YData',RectangleCorners(2,:))

subplot(4,1,4)
plot(squeeze(BraidPoints(1,:,:)),squeeze(BraidPoints(2,:,:)),'o','Color',grayIntensity*[1 1 1])

subplot(4,1,[1 2 3])
plot(squeeze(BraidPoints_Track(1,:,:)),squeeze(BraidPoints_Track(2,:,:)),'o','Color',grayIntensity*[1 1 1])
% pause(0.1)
%% The Braid Path

% Generate an N braid string to follow, excluding the trivial generator.
Braid = repmat(N-1:-1:1,1,ceil(M/(N-1)));
Braid = [Braid(1:M);zeros(size(Braid(1:M)))];
%[randi(N-1,1,M);zeros(1,M)];

% % % An M = 80 randomly generated braid string:
% % Braid = ...
% % [2  4  3  3  4  2  4  4  2  3  1  1  3  4  4  1  3  2  1  2 ...
% %  1  4  2  3  1  3  2  3  3  3  2  1  1  4  1  4  3  4  1  2 ...
% %  1  4  1  4  4  4  1  2  2  4  2  4  1  2  1  1  4  3  3  1 ... 
% %  4  3  2  3  2  1  1  1  1  1  2  1  4  4  2  2  2  4  2  1];

% Compute the node each agent will be at each step of the braid.
BraidAgents = repmat((1:N)',1,M+1); % Start with the trivial generator
for i = 2:M
    BraidHat = Braid(1,i);
    % Swap agent sigma(i) and sigma(i)+1
    BraidAgents(:,i) = BraidAgents(:,i-1)...
        -(BraidAgents(BraidHat,i-1)*((1:N).'==BraidHat)+BraidAgents(BraidHat+1,i-1)*((1:N).'==BraidHat+1))...
        +(BraidAgents(BraidHat+1,i-1)*((1:N).'==BraidHat)+BraidAgents(BraidHat,i-1)*((1:N).'==BraidHat+1));
    if N>=5
        if(BraidHat == 1)
            BraidHat = 2 + 1;%randi(2);
        elseif(BraidHat==2)
            BraidHat = 4;
        elseif(BraidHat == 3)
            BraidHat = 1;
        elseif(BraidHat == 4)
        BraidHat = 3 - 1;%randi(2);
        end
        Braid(2,i) = BraidHat;
        BraidAgents(:,i) = BraidAgents(:,i)...
            -(BraidAgents(BraidHat,i)*((1:N).'==BraidHat)+BraidAgents(BraidHat+1,i)*((1:N).'==BraidHat+1))...
            +(BraidAgents(BraidHat+1,i)*((1:N).'==BraidHat)+BraidAgents(BraidHat,i)*((1:N).'==BraidHat+1));

    end
end

BraidPoint_X = squeeze(BraidPoints(1,:,1:end));
BraidPoint_Y = squeeze(BraidPoints(2,:,1:end));
BraidPoint_Track_X = squeeze(BraidPoints_Track(1,:,1:end));
BraidPoint_Track_Y = squeeze(BraidPoints_Track(2,:,1:end));


for i = 1:N
    subplot(4,1,4)
    plot(BraidPoint_X(BraidAgents==i),...
        BraidPoint_Y(BraidAgents==i),'Color',c{i},'linewidth',2)
    subplot(4,1,[1 2 3])
    plot(BraidPoint_Track_X(BraidAgents==i),...
        BraidPoint_Track_Y(BraidAgents==i),'Color',c{i},'linewidth',2)
end
% pause
pause(0.1)
%% Setting up the simulation

% Angular Velocity Tuning Gain
K_Theta = 2*5;

% % % Simulation time
% % dt = 0.001; % 0.1 ms
% % 
% % % Terminal time
% % tf = 30;%2*M; % 2 seconds per braid step

% Time Vector 
t = 0:dt:tf;


% Agent Radius
r_agents = 0.07/2; % 7/2 cm

% Safety separation between agents
d_safe = 2*r_agents*1.1;% Twice their radius plus 10 percent satfety margin

% Allocating position and heading arrays for the Agents
X = zeros(N,length(t)); 
Y = zeros(N,length(t));
THETA = zeros(N,length(t));
% Allocate Memory
X_Straightened = zeros(N,length(t));
Y_Straightened = zeros(N,length(t));

% Allocate vector of linear velocities for N Agents, 2 per braid step.
V = zeros(N,2*M); 
% Allocate vector of switching times for N Agents, 2 per braid step.
TAU = zeros(N,2*M);

% Initializing Agent Information
for j = 1:N
    if(sum(abs(diff((BraidAgents(:,1)==j)-(BraidAgents(:,2)==j))))~=0)
        X(j,1) = [1,0]*T(HH_Braid(:,:,abs(diff((BraidAgents(:,1)==j)-(BraidAgents(:,2)==j)))==2,1),squeeze(BraidPoints(:,j,1)));
        Y(j,1) = [0,1]*T(HH_Braid(:,:,abs(diff((BraidAgents(:,1)==j)-(BraidAgents(:,2)==j)))==2,1),squeeze(BraidPoints(:,j,1)));
    else
        X(j,1) = squeeze(BraidPoints_Track(1,j,1));
        Y(j,1) = squeeze(BraidPoints_Track(2,j,1));        
    end
end
% Distribute T evenly for now... to determine time alloted per braid step
tf_braid = tf/M; % seconds

%%%%%%%%%%%%%%%%%% Compute the braid controller %%%%%%%%%%%%%%%%%%
ii = 1;
jj = 1;
% HH_Braid = circshift(HH_Braid,[0 0 0 -1]);
for i = 1:M;
    if((BraidPoints(1,1,i)>StraigthenedPath_InnerBoundary(1,ii+1)))
        if(ii <= size(StraigthenedPath_InnerBoundary,2))
            if((BraidPoints(1,1,i)>StraigthenedPath_InnerBoundary(1,ii+2)))
                fprintf('Exceeded the first transform range.\n')
                fprintf('Increase number of braid points, decrease number of trapezoids, or rework the code to make it work (you lazy bum...)\n')
                error('Skipped a transform')
            end
        end
        ii = ii + 1;
    end
    if((BraidPoints(1,1,i+1)>StraigthenedPath_InnerBoundary(1,jj+1)))
        if(jj <= size(StraigthenedPath_InnerBoundary,2))
            if((BraidPoints(1,1,i+1)>StraigthenedPath_InnerBoundary(1,jj+2)))
                fprintf('Exceeded the first transform range.\n')
                fprintf('Increase number of braid points, decrease number of trapezoids, or rework the code to make it work (you lazy bum...)\n')
                error('Skipped a transform')
            end
        end
        jj = jj + 1;
    end
    AGENTS = 0;
    for j = 1:N;
        % Check if we have computed the controller for agent j.
        if(sum(AGENTS==j)==0)
            % Get four braid points to compute parameters (unless we go
            % straight, in which case we only get two) (Note the order:
            % connecting lines are subsequent)
            BraidCorners = [BraidPoints(:,BraidAgents(:,i)==j,i),BraidPoints(:,BraidAgents(:,i+1)==j,i+1)];
            
            if(sum(((1:N)').*(BraidAgents(:,i)==j))==sum(((1:N)').*(BraidAgents(:,i+1)==j)))
                % Agent j is not interacting with any one, set constant
                % velocity to reach other end in at time tf.
                Braid_j = abs(diff(BraidAgents(:,i)==j));
                V(j,[2*i-1 2*i]) = norm((BraidCorners(1,2)-BraidCorners(1,1)))/tf_braid.*[1 1];
                TAU(j,[2*i-1 2*i]) = [tf_braid/2+tf_braid*(i-1), tf_braid*i];
                AGENTS = [AGENTS,j];
            elseif(sum(((1:N)').*(BraidAgents(:,i)==j))<sum(((1:N)').*(BraidAgents(:,i+1)==j)))
                % Agent j is interacting with the agent below, set the corners
                % appropriately
                BraidOver = 1;
                BraidCorners = [BraidCorners,BraidPoints(:,BraidAgents(:,i+1)==j,i),BraidPoints(:,BraidAgents(:,i)==j,i+1)];
            elseif(sum(((1:N)').*(BraidAgents(:,i)==j))>sum(((1:N)').*(BraidAgents(:,i+1)==j)))
                % Agent j is interacting with the agent above, set the corners
                % appropriately
                BraidOver = 0;
                BraidCorners = [BraidCorners,BraidPoints(:,BraidAgents(:,i+1)==j,i),BraidPoints(:,BraidAgents(:,i)==j,i+1)];
            end
            Braid_j = abs(diff((BraidAgents(:,i)==j)-(BraidAgents(:,i+1)==j)))==2;
            %% Compute the control if this agent is interacting with another
            if(size(BraidCorners,2)>2)
                % Which agent is assigned to interact with agent j at this braid
                NeighborAgent = sum(BraidAgents(:,i).*(BraidAgents(:,i+1)==j));

                %{
                %                 % Check whether or not we are at the boundary of a
                %                 % transform
                %                 if(ii~=jj) % We are at a boundary, unfortunately we need to compute the control in the real world due to parameterization discontinuities
                %
                %                     % Find the intersection point in real world coordinates
                %                     % by transforming the braid points
                %                     p_World = (T(HH(:,:,jj),BraidCorners(:,2))-T(HH(:,:,ii),BraidCorners(:,1)))*([1,0]*([(T(HH(:,:,jj),BraidCorners(:,2))-T(HH(:,:,ii),BraidCorners(:,1))),-(T(HH(:,:,jj),BraidCorners(:,4))-T(HH(:,:,ii),BraidCorners(:,3)))]\(T(HH(:,:,ii),BraidCorners(:,3))-T(HH(:,:,ii),BraidCorners(:,1)))))+T(HH(:,:,ii),BraidCorners(:,1));
                %
                %                     % Compute the saftey boundary points from p
                %                     Normal_X = (T(HH(:,:,jj),BraidCorners(:,2))-T(HH(:,:,ii),BraidCorners(:,1)))/norm(T(HH(:,:,jj),BraidCorners(:,2))-T(HH(:,:,ii),BraidCorners(:,1)));
                %                     Normal_Y = (T(HH(:,:,jj),BraidCorners(:,4))-T(HH(:,:,ii),BraidCorners(:,3)))/norm(T(HH(:,:,jj),BraidCorners(:,4))-T(HH(:,:,ii),BraidCorners(:,3)));
                %
                %                     x1_World = p_World-d_safe/sin(acos(Normal_X.'*Normal_Y))*Normal_X;
                %                     x2_World = p_World+d_safe/sin(acos(Normal_X.'*Normal_Y))*Normal_X;
                %                     y1_World = p_World-d_safe/sin(acos(Normal_X.'*Normal_Y))*Normal_Y;
                %                     y2_World = p_World+d_safe/sin(acos(Normal_X.'*Normal_Y))*Normal_Y;
                %
                %                     %%%%%%%%%%%%%%%%%%%%%%% USELESS %%%%%%%%%%%%%%%%%%%%%%%
                %                     % % Transform these points onto the straigthened track
                %                     % x1 = T(HH(:,:,ii+(W(:,ii).'*[x1_World;1]>0)),x1_World);
                %                     % x2 = T(HH(:,:,ii+(W(:,ii).'*[x2_World;1]>0)),x2_World);
                %                     % y1 = T(HH(:,:,ii+(W(:,ii).'*[y1_World;1]>0)),y1_World);
                %                     % y2 = T(HH(:,:,ii+(W(:,ii).'*[y2_World;1]>0)),y2_World);
                %                     %%%%%%%%%%%%%%%%%%%%%%% USELESS %%%%%%%%%%%%%%%%%%%%%%%
                %                     % Find the distance to the intersection point along the
                %                     % line from T(BraidCorners(:,1)) and from
                %                     % T(BraidCorners(:,2)).
                %                     p_dist1x_World = norm(p_World-T(HH(:,:,ii),BraidCorners(:,1)));
                %                     p_dist2x_World = norm(p_World-T(HH(:,:,jj),BraidCorners(:,2)));
                %
                %                     % Find the distance to the intersection point along the
                %                     % line from BraidCorners(:,1) and from BraidCorners(:,2).
                %                     p_dist1y = norm(p_World-BraidCorners(:,3));
                %                     p_dist2y = norm(p_World-BraidCorners(:,4));
                %
                %                 else % It is safe to compute the controller on the transformed space, proceed to do so
                %}
                % Find the intersection point
                p = (BraidCorners(:,2)-BraidCorners(:,1))*([1,0]*([(BraidCorners(:,2)-BraidCorners(:,1)),-(BraidCorners(:,4)-BraidCorners(:,3))]\(BraidCorners(:,3)-BraidCorners(:,1))))+BraidCorners(:,1);
                % Find the distance to the intersection point along the
                % line from BraidCorners(:,1) and from BraidCorners(:,2).
                p_dist1x = norm(p-BraidCorners(:,1));
                p_dist2x = norm(p-BraidCorners(:,2));
                
                % Find the distance to the intersection point along the
                % line from BraidCorners(:,1) and from BraidCorners(:,2).
                p_dist1y = norm(p-BraidCorners(:,3));
                p_dist2y = norm(p-BraidCorners(:,4));
                
                % Compute the saftey boundary points from p
                Normal_X = (T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2))-T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)))/norm(T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2))-T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)));
                Normal_Y = (T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,4))-T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,3)))/norm(T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,4))-T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,3)));
                
                %%%%%%%%%%%%%%% Plotting for debugging %%%%%%%%%%%%%%%
                %{
                % subplot(4,1,[1 2 3])
                % plot([1,0]*T((HH_Braid(:,:,Braid_j,i)),BraidCorners(:,2)),[0,1]*T((HH_Braid(:,:,Braid_j,i)),BraidCorners(:,2)),'ro','markersize',10)
                % quiver([1,0]*T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)),[0,1]*T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)),Normal_X(1),Normal_X(2))
                % quiver([1,0]*T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2)),[0,1]*T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2)),-Normal_X(1),-Normal_X(2))
                
                % plot([1,0]*T((HH(:,:,jj)),BraidCorners(:,2)),[0,1]*T((HH(:,:,jj)),BraidCorners(:,2)),'ro','markersize',10)
                % plot([1,0]*T((HH(:,:,ii)),BraidCorners(:,1)),[0,1]*T((HH(:,:,ii)),BraidCorners(:,1)),'ro','markersize',10)
                % plot([1,0]*T((HH(:,:,ii)),BraidCorners(:,3)),[0,1]*T((HH(:,:,ii)),BraidCorners(:,3)),'ro','markersize',10)
                % plot([1,0]*T((HH(:,:,jj)),BraidCorners(:,4)),[0,1]*T((HH(:,:,jj)),BraidCorners(:,4)),'ro','markersize',10)
                % quiver([1,0]*T((HH(:,:,ii)),BraidCorners(:,1)),[0,1]*T((HH(:,:,ii)),BraidCorners(:,1)),Normal_X(1),Normal_X(2))
                % quiver([1,0]*T((HH(:,:,jj)),BraidCorners(:,2)),[0,1]*T((HH(:,:,jj)),BraidCorners(:,2)),-Normal_X(1),-Normal_X(2))
                % plot(x1_World(1),x1_World(2),'ro')
                % plot(x2_World(1),x2_World(2),'ro')
                % subplot(4,1,4)
                % plot(x1(1),x1(2),'ro')
                % plot(x2(1),x2(2),'ro')
                % plot(p_World(1),p_World(2),'Marker','p')
                %}
                
                %
                p_World = T(HH_Braid(:,:,Braid_j,i),p);%(T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2))-T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)))*([1,0]*([(T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2))-T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1))),-(T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,4))-T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,3)))]\(T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,3))-T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)))))+T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1));
                
                x1_World = p_World-d_safe/sin(acos(Normal_X.'*Normal_Y))*Normal_X;
                x2_World = p_World+d_safe/sin(acos(Normal_X.'*Normal_Y))*Normal_X;
                y1_World = p_World-d_safe/sin(acos(Normal_X.'*Normal_Y))*Normal_Y;
                y2_World = p_World+d_safe/sin(acos(Normal_X.'*Normal_Y))*Normal_Y;
                
                %                     if(p_World(1)>StraigthenedPath_InnerBoundary(1,ii+1))
                %
                %                     else
                x1 = T(inv(HH_Braid(:,:,Braid_j,i)),x1_World);
                x2 = T(inv(HH_Braid(:,:,Braid_j,i)),x2_World);
                y1 = T(inv(HH_Braid(:,:,Braid_j,i)),y1_World);
                y2 = T(inv(HH_Braid(:,:,Braid_j,i)),y2_World);
                
                %                     end
                
                
                % x1 = T(inv(HH(:,:,ii)),x1_World);
                % x2 = T(inv(HH(:,:,ii)),x2_World);
                % y1 = T(inv(HH(:,:,ii)),y1_World);
                % y2 = T(inv(HH(:,:,ii)),y2_World);
                
                % Compute the distance from p to safety region.
                delta_x = norm(p-BraidOver*x2-(1-BraidOver)*x1);
                delta_y = norm(p-(1-BraidOver)*y2-BraidOver*y1);
            end
            % Check whether we are unable to achieve this braid without
            % collisions
            if(p_dist1x*(1-BraidOver)+p_dist2x*BraidOver-delta_x<=0)
                fprintf('The controller cannot achieve this braid collision free without stopping or going backwards.\n')
                fprintf('This was found when attempting to compute the controller for agent %i at braid step %i.\n',j,i)
                fprintf('Terminating simulation...\n')
                % Plot the danger zone
                subplot(4,1,[1 2 3])
                plot([1,0]*T((HH_Braid(:,:,Braid_j,i)),p),[0,1]*T((HH_Braid(:,:,Braid_j,i)),p),'Marker','d')
                plot(d_safe/sin(acos(Normal_X.'*Normal_Y))*cos(2*pi*(0:0.001:1))+[1,0]*T((HH_Braid(:,:,Braid_j,i)),p),...
                     d_safe/sin(acos(Normal_X.'*Normal_Y))*sin(2*pi*(0:0.001:1))+[0,1]*T((HH_Braid(:,:,Braid_j,i)),p),'--b')
                plot([1,0]*[T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)),T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2))],...
                     [0,1]*[T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)),T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2))],'Color',[0.5 0 1],'Marker','x')
                plot([1,0]*[T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,3)),T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,4))],...
                     [0,1]*[T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,3)),T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,4))],'Color',[1 0.5 0],'Marker','x')
                error('Non-Positive Velocity')
            end
            if(p_dist1y*BraidOver+p_dist2y*(1-BraidOver)-delta_y<=0)
                fprintf('The controller cannot achieve this braid collision free without stopping or going backwards.\n')
                fprintf('This was found when attempting to compute the controller for agent %i at braid step %i.\n',NeighborAgent,i)
                fprintf('Terminating simulation...\n')
                % Plot the danger zone
                subplot(4,1,[1 2 3])
                plot(p_World(1),p_World(2),'Marker','d')
                plot(d_safe/sin(acos(Normal_X.'*Normal_Y))*cos(2*pi*(0:0.001:1))+[1,0]*T((HH_Braid(:,:,Braid_j,i)),p),...
                     d_safe/sin(acos(Normal_X.'*Normal_Y))*sin(2*pi*(0:0.001:1))+[0,1]*T((HH_Braid(:,:,Braid_j,i)),p),'--b')
                error('Non-Positive Velocity')
            end
            % Compute the distance from the initial point to the switch and
            % to the final point
            Switch_dist = [p_dist1x+delta_x*BraidOver-delta_x*(1-BraidOver);
                           p_dist1y+delta_y*(1-BraidOver)-delta_y*(BraidOver)];
            Total_dist = [p_dist1x+p_dist2x;
                          p_dist1y+p_dist2y];
            
            % Compute the switch time
            if(abs(norm(Total_dist-Switch_dist)^2-norm(Switch_dist)^2)<=1e-7||1)
                % There is horizontal symmetry, switch time is  0.5*tf_braid...
                tau = 0.5*tf_braid;
            else
                tau = ((norm(Total_dist-Switch_dist)-norm(Switch_dist)^2)/(norm(Total_dist-Switch_dist)^2-norm(Switch_dist)^2))*tf_braid;
            end
            if(tau<=0||tau>=tf_braid)
                fprintf('Something is fishy with the switch time at braid step %i for agents %i and %i.\n',i,j,NeighborAgent);
                fprintf('The computed tau was %f but it should lie in the interval (0,%f).\n',tau,tf_braid);
                fprintf('Terminating simulation...\n')
                
                subplot(4,1,[1 2 3])
                plot(p_World(1),p_World(2),'Marker','d')
                plot(d_safe/sin(acos(Normal_X.'*Normal_Y))*cos(2*pi*(0:0.001:1))+p_World(1),...
                     d_safe/sin(acos(Normal_X.'*Normal_Y))*sin(2*pi*(0:0.001:1))+p_World(2),'--b')
                
                plot([1,0]*[T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)),T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2))],...
                     [0,1]*[T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,1)),T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,2))],...
                     'Color',[0.5 0 1],'Marker','x')
                
                plot([1,0]*[T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,3)),T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,4))],...
                      [0,1]*[T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,3)),T(HH_Braid(:,:,Braid_j,i),BraidCorners(:,4))],...
                      'Color',[1 0.5 0],'Marker','x')
                
                subplot(4,1,4)
                plot(p(1),p(2),'Marker','d')
                % plot(d_safe/sin(acos(Normal_X.'*Normal_Y))*cos(2*pi*(0:0.001:1))+[1,0]*T((HH(:,:,ii)),p),d_safe/sin(acos(Normal_X.'*Normal_Y))*sin(2*pi*(0:0.001:1))+[0,1]*T((HH(:,:,ii)),p),'--b')
                plot([BraidCorners(1,1),BraidCorners(1,2)],...
                     [BraidCorners(2,1),BraidCorners(2,2)],'Color',[0.5 0 1],'Marker','x')
                plot([BraidCorners(1,3),BraidCorners(1,4)],...
                     [BraidCorners(2,3),BraidCorners(2,4)],'Color',[1 0.5 0],'Marker','x')
                error('Fishy Tau')
            end
            
            % Store the switching times for this agent and braid.
            TAU([j NeighborAgent],[2*i-1 2*i]) = repmat([tau+tf_braid*(i-1), tf_braid*i],2,1);
            
            % Store the linear velocities for this agent and braid.
            V([j NeighborAgent],[2*i-1 2*i]) = [Switch_dist/tau,(Total_dist-Switch_dist)/(tf_braid-tau)];
            
            % Update the agent's whose controller has been computed.
            AGENTS = [AGENTS,j,NeighborAgent];
            %             end
        end
    end
end
 

%% Run Simulation
Braid_j =1;
M_BraidCounter = 1;
Switch_Time_Counter = ones(1,N);

% Initialize
X_Straightened(:,1) = squeeze(BraidPoints(1,:,1));
Y_Straightened(:,1) = squeeze(BraidPoints(2,:,1));
HeadingTrack = zeros(2,N,length(t));
HeadingStraightened = zeros(2,N,length(t));
for j = 1:N
    HeadingTrack(:,j,1) = ((BraidPoints_Track(:,BraidAgents(:,2)==j,2)...
            -BraidPoints_Track(:,BraidAgents(:,1)==j,1))/...
            norm(BraidPoints_Track(:,BraidAgents(:,2)==j,2)...
            -BraidPoints_Track(:,BraidAgents(:,1)==j,1)));
    HeadingStraightened(:,j,1)  = ((BraidPoints(:,BraidAgents(:,2)==j,2)...
            -BraidPoints(:,BraidAgents(:,1)==j,1))/...
            norm(BraidPoints(:,BraidAgents(:,2)==j,2)...
            -BraidPoints(:,BraidAgents(:,1)==j,1)));
end
% HeadingTrack(:,:,1) = [cos(THETA(:,1))';sin(THETA(:,1))'];
% HeadingStraightened(:,:,1) = [cos(THETA(:,1))';sin(THETA(:,1))'];
% keyboard
% old_M_Braid_Counter = ones(1,N);
for i = 1:length(t)-1
    for j = 1:N
        % At each path segment, compute the segment
        if(t(i)>TAU(j,Switch_Time_Counter(j)))
            % Time to switch velocities
            Switch_Time_Counter(j) = Switch_Time_Counter(j) + 1;
            % % % % Update M_BraidCounter every two Switch_Time_Counter updates
            % % % M_BraidCounter(j) = M_BraidCounter(j) + mod(Switch_Time_Counter(j)+1,2);
            % % % if(M_BraidCounter(j)>M-1)
            % % %     M_BraidCounter(j) = M-1;
            % % % end
            if(Switch_Time_Counter(j)>(2*M))
                Switch_Time_Counter(j) = 2*M;
            end
%             keyboard
        end
        % % % if(sum(abs(diff((BraidAgents(:,M_BraidCounter(j))==j)-(BraidAgents(:,M_BraidCounter(j)+1)==j)))==2)~=0)
        % % %     Braid_j = abs(diff((BraidAgents(:,M_BraidCounter(j))==j)-(BraidAgents(:,M_BraidCounter(j)+1)==j)))==2;
        % % % else
        % % %     Braid_j = abs(diff(BraidAgents(:,M_BraidCounter(j))==j))==1;
        % % % end
        
        % Compute the dynamics for each agent
        % Linear velocity components are on the straightened track
        dx = V(j,Switch_Time_Counter(j))*HeadingStraightened(1,j,i);%cos(THETA(j,i));
        dy = V(j,Switch_Time_Counter(j))*HeadingStraightened(2,j,i);%sin(THETA(j,i));
        % % % % Compute angular velocity on the track (real world)
        % % % dtheta = K_Theta*[-sin(THETA(j,i)),cos(THETA(j,i))]*...
        % % %     (BraidPoints_Track(:,BraidAgents(:,(M_BraidCounter(j)+1))==j,M_BraidCounter(j)+1)...
        % % %     -BraidPoints_Track(:,BraidAgents(:,M_BraidCounter(j))==j,M_BraidCounter(j)))/...
        % % %     norm(BraidPoints_Track(:,BraidAgents(:,(M_BraidCounter(j)+1))==j,M_BraidCounter(j)+1)...
        % % %     -BraidPoints_Track(:,BraidAgents(:,M_BraidCounter(j))==j,M_BraidCounter(j)));
%         keyboard
        % Determine which braid step we are in based on robot position
        FirstTransformFound = 0;
        BreakFlag = 0;
        for ii = 1:M
            for jj = 1:N-1
               if(inpolygon(X(j,i),Y(j,i),TrapezoidPolygons_X_Enlarged(jj+(N-1)*(ii-1),:)',TrapezoidPolygons_Y_Enlarged(jj+(N-1)*(ii-1),:)')&&~FirstTransformFound);%
                   % Braid_j = jj;
                   if(inpolygon(X(j,i),Y(j,i),TrapezoidPolygons_X_Enlarged(jj+(N-1)*(min(ii,M-1)),:)',TrapezoidPolygons_Y_Enlarged(jj+(N-1)*(min(ii,M-1)),:)')&&~FirstTransformFound);%
                       M_BraidCounter = min(ii+1,M);
                       FirstTransformFound = 1;
                       BreakFlag = 1;
                   else
                       M_BraidCounter = ii;
                       FirstTransformFound = 1;
                       BreakFlag = 1;
                   end
               end
            end
            if(BreakFlag == 1);
                break;
            end
        end
        if(sum(abs(diff((BraidAgents(:,M_BraidCounter)==j)-(BraidAgents(:,M_BraidCounter+1)==j)))==2)~=0)
            Braid_j = abs(diff((BraidAgents(:,M_BraidCounter)==j)-(BraidAgents(:,M_BraidCounter+1)==j)))==2;
        else
            Braid_j = abs(diff(BraidAgents(:,M_BraidCounter)==j))==1;
        end
        % % keyboard
        % Use forward Euler method to estimate the new state
        % Get current position in straigthened track
        % Convert the velocities to world plane coordinates
        X(j,i+1) =  X(j,i) +...
            [1,0]*[dTdx1(HH_Braid(:,:,Braid_j,M_BraidCounter),...
                   T(inv(HH_Braid(:,:,Braid_j,M_BraidCounter)),...
                   [X(j,i);Y(j,i)])),...
                   dTdx2(HH_Braid(:,:,Braid_j,M_BraidCounter),...
                   T(inv(HH_Braid(:,:,Braid_j,M_BraidCounter)),...
                   [X(j,i);Y(j,i)]))]*...
                   [dx;dy]*(t(i+1)-t(i));
        Y(j,i+1) =  Y(j,i) +...
            [0,1]*[dTdx1(HH_Braid(:,:,Braid_j,M_BraidCounter),...
                   T(inv(HH_Braid(:,:,Braid_j,M_BraidCounter)),...
                   [X(j,i);Y(j,i)])),...
                   dTdx2(HH_Braid(:,:,Braid_j,M_BraidCounter),...
                   T(inv(HH_Braid(:,:,Braid_j,M_BraidCounter)),...
                   [X(j,i);Y(j,i)]))]*...
                   [dx;dy]*(t(i+1)-t(i));
        % THETA(j,i+1) =  THETA(j,i) + dtheta*(t(i+1)-t(i));
        %% Assume perfect heading control
%         HeadingTrack(:,j,i+1) = ((BraidPoints_Track(:,BraidAgents(:,(M_BraidCounter+1))==j,M_BraidCounter+1)...
%             -BraidPoints_Track(:,BraidAgents(:,M_BraidCounter)==j,M_BraidCounter))/...
%             norm(BraidPoints_Track(:,BraidAgents(:,(M_BraidCounter+1))==j,M_BraidCounter+1)...
%             -BraidPoints_Track(:,BraidAgents(:,M_BraidCounter)==j,M_BraidCounter)));
        HeadingTrack(:,j,i+1) = ((BraidPoints_Track(:,BraidAgents(:,(M_BraidCounter+1))==j,M_BraidCounter+1)...
            -[X(j,i);Y(j,i)])/...
            norm(BraidPoints_Track(:,BraidAgents(:,(M_BraidCounter+1))==j,M_BraidCounter+1)...
            -[X(j,i);Y(j,i)]));
        THETA(j,i+1) =  atan2(HeadingTrack(2),HeadingTrack(1));
       
        
        X_Straightened(j,i+1) = [1,0]*T(inv(HH_Braid(:,:,Braid_j,M_BraidCounter)),[X(j,i+1);Y(j,i+1)]);
        Y_Straightened(j,i+1) = [0,1]*T(inv(HH_Braid(:,:,Braid_j,M_BraidCounter)),[X(j,i+1);Y(j,i+1)]);
%         HeadingStraightened(:,j,i+1) = ((BraidPoints(:,BraidAgents(:,(M_BraidCounter+1))==j,M_BraidCounter+1)...
%             -BraidPoints(:,BraidAgents(:,M_BraidCounter)==j,M_BraidCounter))/...
%             norm(BraidPoints(:,BraidAgents(:,(M_BraidCounter+1))==j,M_BraidCounter+1)...
%             -BraidPoints(:,BraidAgents(:,M_BraidCounter)==j,M_BraidCounter)));
        HeadingStraightened(:,j,i+1) = ((BraidPoints(:,BraidAgents(:,(M_BraidCounter+1))==j,M_BraidCounter+1)...
            -[X_Straightened(j,i);Y_Straightened(j,i)])/...
            norm(BraidPoints(:,BraidAgents(:,(M_BraidCounter+1))==j,M_BraidCounter+1)...
            -[X_Straightened(j,i);Y_Straightened(j,i)]));
%         HeadingStraightened(:,j,i+1) = ...
%             [dTdx1(inv(HH_Braid(:,:,Braid_j,M_BraidCounter)),[X(j,i+1);...
%                                                               Y(j,i+1)]),...
%              dTdx2(inv(HH_Braid(:,:,Braid_j,M_BraidCounter)),[X(j,i+1);...
%                                                               Y(j,i+1)])]...
%                                                              *HeadingTrack(:,j,i+1);   
    end
%     if(sum(M_BraidCounter > old_M_Braid_Counter)==N)
%         old_M_Braid_Counter = 
%     end
end
ElapsedTime = toc;
fprintf('It took %f seconds to evaluate \na %i-agent, %i-length braid with \n%f seconds time steps.\n',ElapsedTime,N,M,dt);
% save('PerfectTheta_MediumM_Smalldt_Run2');
% break
%% Play the simulated results
% Time step for simulation
delta_time = 0.01;
% Compute how many frames are needed for the given time step
NumFrames = ceil(tf/delta_time)+1;
% Preallocate the struct array for the struct returned by getframe
Frames(NumFrames) = struct('cdata',[],'colormap',[]);

%{
% % % % % Compute the heading vector for the robots
% % % % HeadingTrack = zeros(2,N);%length(t));
% % % % HeadingStraightened = zeros(2,N,length(t));
% % % % for j = 1:N
% % % %     if(sum(abs(diff((BraidAgents(:,1)==j)-(BraidAgents(:,2)==j)))==2)~=0)
% % % %         Braid_j = abs(diff((BraidAgents(:,1)==j)-(BraidAgents(:,2)==j)))==2;
% % % %     else
% % % %         Braid_j = abs(diff(BraidAgents(:,1)==j))==1;
% % % %     end
% % % %     HeadingTrack(1,j,1) = r_agents*cos(THETA(j,1));
% % % %     HeadingTrack(2,j,1) = r_agents*sin(THETA(j,1));
% % % %     % Compute the heading in the straightened track using the jacobian of
% % % %     % the transform
% % % %     HeadingStraightened(:,j,1) = ...
% % % %         [dTdx1(HH_Braid(:,:,Braid_j,1),[X_Straightened(j,1);...
% % % %                                         Y_Straightened(j,1)]),...
% % % %          dTdx2(HH_Braid(:,:,Braid_j,1),[X_Straightened(j,1);...
% % % %                                         Y_Straightened(j,1)])]...
% % % %                                         *HeadingTrack(:,j,1);   
% % % % end
%}

% Plot the robots on the curved track
subplot(4,1,[1 2 3])
RobotPlot_Track = quiver(X(:,1),Y(:,1),HeadingTrack(1,:,1)',HeadingTrack(2,:,1)',0,'Marker','o','linewidth',2,'color','k');
% Plot the robots on the straightened track
subplot(4,1,4)
RobotPlot_Straightened = quiver(X_Straightened(:,1),Y_Straightened(:,1),HeadingStraightened(1,:,1)',HeadingStraightened(2,:,1)',0,'Marker','o','linewidth',2,'color','k');
figure(gcf)%,'Position',[100 100 850 600])
Frames(1) = getframe(gcf);
% Starting time
time = 0;
frameCounter = 2;
while time<=tf
    i = find(abs(t-time)<dt*0.8,1);
    subplot(4,1,[1 2 3])
    set(RobotPlot_Track,'XData',X(:,i),'YData',Y(:,i),'UData',r_agents*HeadingTrack(1,:,i)','VData',r_agents*HeadingTrack(2,:,i)')
    subplot(4,1,4)
    set(RobotPlot_Straightened,'XData',X_Straightened(:,i),'YData',Y_Straightened(:,i),'UData',r_agents*HeadingStraightened(1,:,i)','VData',r_agents*HeadingStraightened(2,:,i)')
    Frames(frameCounter) = getframe(gcf);
    frameCounter = frameCounter + 1;
    time = time + delta_time;
    drawnow;
    pause(delta_time/10)
end
break

%% Play the simuated results
% Preallocate the struct array for the struct returned by getframe
delta_time = 0.01;
NumFrames = ceil(tf/delta_time)+1;
frameCounter = 2;
Frames(NumFrames) = struct('cdata',[],'colormap',[]);
% Plot the robots on the track
subplot(4,1,[1 2 3])
RobotPlot_Track = quiver(X(:,1),Y(:,1),r_agents*cos(THETA(:,1)),r_agents*sin(THETA(:,1)),0,'Marker','o','linewidth',2,'color','k');
% subplot(4,1,4)
% % for j = 1:N
%     RobotPlot_Straightened = quiver(X_Straightened(:,1),Y_Straightened(:,1),r_agents*cos(THETA(:,1)),r_agents*sin(THETA(:,1)),0,'Marker','o','filled','linewidth',2,'color','k');
% end
figure(gcf)%,'Position',[100 100 850 600])
Frames(1) = getframe;
time = 0;
while time<=tf
    i = find(abs(t-time)<dt);
    set(RobotPlot_Track,'XData',X(:,i),'YData',Y(:,i),'UData',r_agents*cos(THETA(:,i)),'VData',r_agents*sin(THETA(:,i)))
%     drawnow
    Frames(frameCounter) = getframe;
    frameCounter = frameCounter + 1;
    time = time + delta_time;
end

        % % % Get the right transform index
        % % if((BraidPoints(1,1,M_BraidCounter)>StraigthenedPath_InnerBoundary(1,ii+1)))
        % %     if(ii <= size(StraigthenedPath_InnerBoundary,2))
        % %         if((BraidPoints(1,1,M_BraidCounter)>StraigthenedPath_InnerBoundary(1,ii+2)))
        % %             fprintf('Exceeded the first transform range.\n')
        % %             fprintf('Increase number of braid points, decrease number of trapezoids, or rework the code to make it work (you lazy bum...)\n')
        % %             break
        % %         end
        % %     end
        % %     ii = ii + 1;
        % % end
        %%%%%%%%%%%%%%%%%%%%%%%%%%% Hypothesis %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Because the rectangular plane is symmetric around the center   %
        % vertical, and the braid points where distributed uniformly, the%
        % transformed waypoints will form an equidistant grid vertically %<------WRONG: we do not have uniform distribution vertically =^( 
        % and horizontally, i.e., the transformed waypoints will lie in  %
        % rectangles of equal height and width. We can therefore use only%
        % four sample points to determine the controller for all agents  %
        % and the transformation will change the controller back.        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%% Hypothesis %%%%%%%%%%%%%%%%%%%%%%%%%%%




