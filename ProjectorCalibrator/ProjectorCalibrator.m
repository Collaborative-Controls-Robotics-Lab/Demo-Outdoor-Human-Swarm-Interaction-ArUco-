%% Projector Calibration Script
% Computes the homogeneous coordinate matrix that transforms between real
% world data and matlab figure data.
%% Create a figure
close all
hFig = figure('units','normalized','position',[-1 0 1 1],'MenuBar','none','color','w','WindowState','fullscreen');

%% Create axes
hAxes = axes('Units', 'normalized', 'Position', [0 0 1 1]);
axis(hAxes,[0 1 0 1])
axis manual
axis off
hold on
%% Plot four markers in image coordinates
imageCoordinates = [[0.25 0.9];
                    [0.75 0.9];
                    [0.95 0.3];
                    [0.05 0.3]];
colorMat = 0.85*hsv(4);
hImage = scatter(imageCoordinates(:,1),imageCoordinates(:,2),600,'filled','LineWidth',2,'MarkerFaceColor','Flat','Cdata',colorMat);

%% Add Vicon trackable object
Vicon_path = 'C:\Users\CCRL\Documents\CCRL\Utilities\ViconClient';
addpath(Vicon_path)
disp('Establishing connections with Vicon')
% Initialize the Vicon Client
[ViconClient,numTrackables] = ViconClientInit;
% Fetch initial data to create pose object
viconPose = ViconClientFetch(ViconClient);
% Get the names of the trackable objects
viconNames = {viconPose.name};


%% Prompt user to get world coordinate data
worldCoordinates = zeros(4,2);
for ii = 1:4
    d = dialog('Position',[300 300 500 300],'Name','Move Robot');

    txt = uicontrol('Parent',d,...
               'Style','text',...
               'Position',[0 0 500 300],...
               'String',{['Move ',viconNames{1},' to the appropriate color spot'];'';'Click the close button when you''re done.'},...
               'BackgroundColor',colorMat(ii,:),...
               'HorizontalAlignment','center',...
               'FontSize',20);
    btn = uicontrol('Parent',d,...
               'Position',[175 40 150 35],...
               'String','Close',...
               'Callback','delete(gcf)','FontSize',20);
    waitfor(d)
    
    disp('Getting position data from Vicon...')
    viconPose = ViconClientFetch(ViconClient,viconPose,1);
    worldCoordinates(ii,:) = -reshape(viconPose(1).positionXYZ(1:2),1,2);
    
end

%% Compute transformation matrix

A = [[worldCoordinates(1,:) 1 0 0 0;
     0 0 0 worldCoordinates(1,:) 1;
     worldCoordinates(2,:) 1 0 0 0;
     0 0 0 worldCoordinates(2,:) 1;
     worldCoordinates(3,:) 1 0 0 0;
     0 0 0 worldCoordinates(3,:) 1;
     worldCoordinates(4,:) 1 0 0 0;
     0 0 0 worldCoordinates(4,:) 1],...
     -reshape(repmat(reshape(imageCoordinates.',[],1),2,1),[],2).*reshape(repmat(reshape(worldCoordinates,[],1),1,2).',[],2)];
b = reshape(imageCoordinates.',[],1);
% The solutions in matrix form
H = reshape([(A\b);1],3,3)';

% The transformation at a point x is given through H by
T = @(H,x) [eye(2) zeros(2,1)]*(H*[x;1]/(H(3,:)*[x;1]));

utilitiesPath = 'C:\Users\CCRL\Documents\CCRL\Utilities\Robotarium\utilities\misc';
addpath(utilitiesPath)
transformFilename = unique_filename('projectiveTransform');
save(transformFilename,'H','T')
% savefig(transformFilename(1:end-4),'hFig');

%% Test Transformation
disp('Testing transformation')
hFigStopLoop = stopLoop;
hPlot = plot(nan,nan,'h','MarkerSize',20);
while true
    if hFigStopLoop.Stop()
        break
    end
    % Get current pose
    viconPose = ViconClientFetch(ViconClient,viconPose,1);
    currentPose = reshape(viconPose(1).positionXYZ(1:2),1,2);
    plotData = T(H,currentPose(:));
    set(hPlot,'XData',plotData(1),'YData',plotData(2));
    drawnow limitrate
end
hFigStopLoop.Clear();
clear hFigStopLoop;
disp('Disconnecting Vicon client')
ViconClientDisconnect(ViconClient)
