%% SIMULATEFROMDATA Simulate the braid from stored data
close all
% Load in stored data
% load('MovieRun');
% load('PerfectTheta_MediumM_Smalldt_Run');
load('C:\Users\yancy_000\Documents\Braiding Videos\Data\BraidDataM80N5_PerfectThetaControl.mat')
% Plot the track
TrackPlot = figure('Name','Braid Track','NumberTitle','off','Position',[11 46 800 450]);%830 770]);%1580 767]); %[-1279 -123 1280 947]);% [3201 -179 1920 1003]
subplot(4,3,[1 2 4 5 7 8])
grid on
hold on

% Some colors in a cell, e.g. for plotting in loops
grayIntensity = 0.3;
c = {'g','r','m','b','c','k','y'};

% Define the width and height of track in decimeters, convert to meters
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

plot(OuterTrack(1,:),OuterTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1],'Marker','+');
plot(InnerTrack(1,:),InnerTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1],'Marker','+');

plot(OuterTrack(1,1),OuterTrack(2,1),'lineStyle','none','Color','r','Marker','+');
plot(InnerTrack(1,1),InnerTrack(2,1),'lineStyle','none','Color','r','Marker','+');
set(gca,'fontsize',14)

subplot(4,3,[3 6 9])
grid on
hold on
plot([0 0 OuterBox(1) OuterBox(1) 0],[0 OuterBox(2) OuterBox(2) 0 0],'-y','linewidth',3)

set(gca,'PlotBoxAspectRatio',[1 1 1])
set(gca,'DataAspectRatio',[1 1 1])
title('Zoomed in Track (Real World)','fontsize',14)
xlabel('East (m)','fontsize',14)
ylabel('North (m)','fontsize',14)
set(gca,'xlim',[-1 1]*0.5+1.5,'ylim',[-1 1]*0.5+0.3)

plot(OuterTrack(1,:),OuterTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1],'Marker','+');
plot(InnerTrack(1,:),InnerTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1],'Marker','+');

plot(OuterTrack(1,1),OuterTrack(2,1),'lineStyle','none','Color','r','Marker','+');
plot(InnerTrack(1,1),InnerTrack(2,1),'lineStyle','none','Color','r','Marker','+');

set(gca,'fontsize',14)

subplot(4,3,[10 11 12])
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
set(gca,'DataAspectRatio',[1 0.5 1])
title('Straightened Track (Virtual World)','fontsize',14)
xlabel('Counter Clockwise along Path (m, along Center of Path)','fontsize',14)
ylabel({'From Outer to Inner'; 'Track Boundary (m)'},'fontsize',14)
axis manual
set(gca,'fontsize',14)


subplot(4,3,[10 11 12])
plot(squeeze(BraidPoints(1,:,:)),squeeze(BraidPoints(2,:,:)),'o','Color',grayIntensity*[1 1 1])

subplot(4,3,[1 2 4 5 7 8])
plot(squeeze(BraidPoints_Track(1,:,:)),squeeze(BraidPoints_Track(2,:,:)),'o','Color',grayIntensity*[1 1 1])

subplot(4,3,[3 6 9])
plot(squeeze(BraidPoints_Track(1,:,:)),squeeze(BraidPoints_Track(2,:,:)),'o','Color',grayIntensity*[1 1 1])

for i = 1:N
    subplot(4,3,[10 11 12])
    plot(BraidPoint_X(BraidAgents==i),...
        BraidPoint_Y(BraidAgents==i),'Color',c{i},'linewidth',2)
    subplot(4,3,[1 2 4 5 7 8])
    plot(BraidPoint_Track_X(BraidAgents==i),...
        BraidPoint_Track_Y(BraidAgents==i),'Color',c{i},'linewidth',2)
    subplot(4,3,[3 6 9])
    plot(BraidPoint_Track_X(BraidAgents==i),...
        BraidPoint_Track_Y(BraidAgents==i),'Color',c{i},'linewidth',2)
end

%% Play the simulated results
% Time step for simulation
delta_time = 0.01;
% Compute how many frames are needed for the given time step
NumFrames = ceil(tf/delta_time)+1;
% Preallocate the struct array for the struct returned by getframe
Frames(NumFrames) = struct('cdata',[],'colormap',[]);

% % % % % % % % Compute the heading vector for the robots
% % % % % % % HeadingTrack = zeros(2,N);%length(t));
% % % % % % % HeadingStraightened = zeros(2,N,length(t));
% % % % % % % for j = 1:N
% % % % % % %     if(sum(abs(diff((BraidAgents(:,1)==j)-(BraidAgents(:,2)==j)))==2)~=0)
% % % % % % %         Braid_j = abs(diff((BraidAgents(:,1)==j)-(BraidAgents(:,2)==j)))==2;
% % % % % % %     else
% % % % % % %         Braid_j = abs(diff(BraidAgents(:,1)==j))==1;
% % % % % % %     end
% % % % % % %     HeadingTrack(1,j,1) = r_agents*cos(THETA(j,1));
% % % % % % %     HeadingTrack(2,j,1) = r_agents*sin(THETA(j,1));
% % % % % % %     % Compute the heading in the straightened track using the jacobian of
% % % % % % %     % the transform
% % % % % % %     HeadingStraightened(:,j,1) = ...
% % % % % % %         [dTdx1(HH_Braid(:,:,Braid_j,1),[X_Straightened(j,1);...
% % % % % % %                                         Y_Straightened(j,1)]),...
% % % % % % %          dTdx2(HH_Braid(:,:,Braid_j,1),[X_Straightened(j,1);...
% % % % % % %                                         Y_Straightened(j,1)])]...
% % % % % % %                                         *HeadingTrack(:,j,1);   
% % % % % % % end
% Plot the robots on the curved track
subplot(4,3,[1 2 4 5 7 8])
% RobotPlot_Track = quiver(X(:,1),Y(:,1),HeadingTrack(1,:,1)',HeadingTrack(2,:,1)',0,'Marker','o','linewidth',2,'color','k');
i=1;
headingX = (X(:,min(i+1,end))-X(:,i))./(sum([(X(:,min(i+1,end))-X(:,i))';(Y(:,min(i+1,end))-Y(:,i))'].^2).^0.5)';
headingY = (Y(:,min(i+1,end))-Y(:,i))./(sum([(X(:,min(i+1,end))-X(:,i))';(Y(:,min(i+1,end))-Y(:,i))'].^2).^0.5)';
RobotPlot_Track = quiver(X(:,1),Y(:,1),(X(:,2)-X(:,1))./(sum([(X(:,2)-X(:,1))';(Y(:,2)-Y(:,1))'].^2).^0.5)',(Y(:,2)-Y(:,1))./(sum([(X(:,2)-X(:,1))';(Y(:,2)-Y(:,1))'].^2).^0.5)',0,'Marker','o','linewidth',2,'color','k');
subplot(4,3,[3 6 9])
RobotPlot_Track2 = quiver(X(:,1),Y(:,1),(X(:,2)-X(:,1))/norm(X(:,2)-X(:,1)),(Y(:,2)-Y(:,1))/norm(Y(:,2)-Y(:,1)),0,'Marker','o','linewidth',2,'color','k');
% RobotPlot_Track2 = quiver(X(:,1),Y(:,1),HeadingTrack(1,:,1)',HeadingTrack(2,:,1)',0,'Marker','o','linewidth',2,'color','k');
% Plot the robots on the straightened track
subplot(4,3,[10 11 12])
RobotPlot_Straightened = quiver(X_Straightened(:,1),Y_Straightened(:,1),HeadingStraightened(1,:,1)',HeadingStraightened(2,:,1)',0,'Marker','o','linewidth',2,'color','k');
figure(gcf)%,'Position',[100 100 850 600])
Frames(1) = getframe(gcf);
% Starting time
% time = 9.53;
time = 0;
r_agents = r_agents*1.5;
frameCounter = 2;
while time<=tf%5+40*dt%9.53+40*dt%tf
    i = find(abs(t-time)<dt*0.8,1);
    headingX(:,2) = mean([(X(:,min(i+1,end))-X(:,i))./(sum([(X(:,min(i+1,end))-X(:,i))';(Y(:,min(i+1,end))-Y(:,i))'].^2).^0.5)' headingX],2);
    headingY(:,2) = mean([(Y(:,min(i+1,end))-Y(:,i))./(sum([(X(:,min(i+1,end))-X(:,i))';(Y(:,min(i+1,end))-Y(:,i))'].^2).^0.5)' headingY],2);
    headingX(:,1) = headingX(:,2);
    headingY(:,1) = headingY(:,2);
    subplot(4,3,[1 2 4 5 7 8])
    set(RobotPlot_Track,'XData',X(:,i),'YData',Y(:,i),'UData',r_agents*headingX(:,2),'VData',r_agents*headingY(:,2))
%     set(RobotPlot_Track,'XData',X(:,i),'YData',Y(:,i),'UData',r_agents*HeadingTrack(1,:,i)','VData',r_agents*HeadingTrack(2,:,i)')
    subplot(4,3,[3 6 9])
    set(RobotPlot_Track2,'XData',X(:,i),'YData',Y(:,i),'UData',r_agents*headingX(:,2),'VData',r_agents*headingY(:,2))
    set(gca,'xlim',[-0.5 0.5]+mean(mean(X(:,max(1,i-100):i))),'ylim',[-0.5 0.5]+mean(mean(Y(:,max(1,i-100):i))))
    subplot(4,3,[10 11 12])
    set(RobotPlot_Straightened,'XData',X_Straightened(:,i),'YData',Y_Straightened(:,i),'UData',r_agents*HeadingStraightened(1,:,i)','VData',r_agents*HeadingStraightened(2,:,i)')
    %%%%%%%%%%
    Frames(frameCounter) = getframe(gcf);
    frameCounter = frameCounter + 1;
drawnow
% pause(dt)
    time = time + delta_time;
end

