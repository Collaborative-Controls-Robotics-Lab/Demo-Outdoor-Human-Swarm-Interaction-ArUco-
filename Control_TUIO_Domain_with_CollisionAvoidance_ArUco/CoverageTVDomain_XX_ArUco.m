%% Demo of Coverage Control over Teleoperated Time-Varying Domains
% Executes coverage control laws over a time-varying domain provided by
% user input
%
% Authors: Xiaotian Xu & Yancy Diaz-Mercado 
%
% Date Created: 04/27/2022
%
% Copyright (c) 2022
% Collaborative Controls and Robotics Laboratory
% University of Maryland, College Park
%
% All rights reserved.

close all, clear all, clc %#ok<CLALL,DUALC>
% Set constant random number generator seed for reproduceability of results
rng(7)
% Flag for simulation/real-robotic experiment
simulate_flag = 0;
metric_plot_flag = 0;
%#ok<*UNRCH> <-- No warning from flags
%% Get Robotarium object used to communicate with the robots/simulator
% Add path to all the utilities
% addpath(genpath(''))
% Set path to Robotarium API
run 'D:\ArUco_Tracking\Utilities\Robotarium\init'

%% Update real robot data
% Set the maximum speed for the computed controller (saturation)
if simulate_flag
    % Number of robots
    numRobots = 4;
    % Define robot object
    r = Robotarium('NumberOfRobots', numRobots, 'ShowFigure', true);
    controlMaxSpeed = 1e-2;
    controlScaleFactor = 1;
else    
    % reference marker id
    refMarkerId = 99;
    % Marker IDs (min --> max; refMarkerId)
    markerIDs = [2 13 53 99].'; % [32  42 236] <<------------------------------------------- Edit this!
    % Corresponding Robots to Markers
    kheperaIDs = ['96' ;'98' ; '92']; %['91' ; '89' ; '97'] <<-------------------------------------- Edit this!
    % Determine number of robots
    numRobots = length(markerIDs)-1;
    %
    disp('Establishing connections with camera')
    %------------------------------------------------------------------
    % Add mexopencv functionalities
    addpath('C:\dev\mexopencv')
    addpath('C:\dev\mexopencv/opencv_contrib');
    % load parameters for ArUco locolization
    load camera_parameters_1280_720_logi.mat -mat camMatrix distCoeffs % <<----- Edit this!
    % load camera_parameters_1280_960_librarycam.mat -mat camMatrix distCoeffs % <<----- Edit this!
    % Marker side length (in meters). Needed for correct scale in camera pose
    markerLength = 0.094; % 0.108; 0.094  % <<--------------------------------------- Edit this!
    dictionaryId = '6x6_250';  % Dictionary id
    showRejected = false;      % Show rejected candidates too
    dictionary = {'Predefined', dictionaryId}; % marker dictionary
    % marker detector parameters
    detectorParams = struct();
    % do corner refinement in markers
    detectorParams.cornerRefinementMethod = 'Subpix';  
    % Open camera object
    vid = cv.VideoCapture(0); % 0 -- open default camera    <<------------------ Edit this!
    vid.FrameWidth = 1280; % Need re-calibration if changed <<------------------ Edit this!
    vid.FrameHeight = 720; % <<------------------------------------------------- Edit this!
    waitTime = 0.01;  % 10 msec
    if ~vid.isOpened()
        error('failed to initialize VideoCapture');
    end
    % Fetch initial localization info for perspective transformation matrix
    totalIterations0 = 0;
    hImg = [];
    LoopFlag0 = true;
    StoreSomeRvecs = cell(10,1);
    StoreSomeTvecs = cell(10,1);
    while LoopFlag0
        % grab frame
        img = vid.read();
        if isempty(img)
            break;
        end
        % detect markers and estimate pose
        [corners, ids, rejected] = cv.detectMarkers(img, dictionary, ...
            'DetectorParameters',detectorParams);
        % return resultant rvecs and tvecs detected by camera for all markers.
        if  ~isempty(ids)
            [rvecs, tvecs] = cv.estimatePoseSingleMarkers(corners, ...
                markerLength, camMatrix, distCoeffs);
        end
        % draw resultant frame captured using camera specs and r/tvecs.
        if ~isempty(ids)
            % draw the boxes and IDs of markers
            img = cv.drawDetectedMarkers(img, corners, 'IDs',ids);
            % draw the x-y-z axis on markers
            for i=1:numel(ids)
                img = cv.drawAxis(img, camMatrix, distCoeffs, ...
                    rvecs{i}, tvecs{i}, markerLength*1.5);
            end
        end
        if showRejected && ~isempty(rejected)
            img = cv.drawDetectedMarkers(img, rejected, 'BorderColor',[255 0 100]);
        end
        if isempty(hImg)
            hImg = imshow(img);
        elseif ishghandle(hImg)
            set(hImg, 'CData',img);
        else
            break;
        end
        drawnow;
        pause(waitTime);
        % we want to get multiple shots of reference marker such that we can get average transformation.
        if length(ids) == numRobots + 1
            refMarkerIndx = ids==refMarkerId;
                StoreSomeRvecs{totalIterations0+1} = rvecs{refMarkerIndx};
                StoreSomeTvecs{totalIterations0+1} = tvecs{refMarkerIndx};
            totalIterations0 = totalIterations0 + 1;
        end
        % Get 10 shots should be fine.
        if totalIterations0>=10
            LoopFlag0 = false;
        end
%         totalIterations0 = totalIterations0 + 1;
%         if totalIterations0>=5 && length(ids) == numRobots+1
%             LoopFlag0 = false;
%         end
    end
    % points (4 corners of a marker) in local frame (a plane).
    objectPts = [-markerLength/2    markerLength/2    markerLength/2   -markerLength/2;...
        markerLength/2    markerLength/2   -markerLength/2   -markerLength/2;...
        0         0         0         0];
    % How big we want a real-world workspace
    halfWidthSpace = 1.8; % <<-------------------- Eidt this
    desiredCoordinates = [-markerLength/2 -halfWidthSpace+markerLength;... % top left
        markerLength/2 -halfWidthSpace+markerLength;... % top right
        markerLength/2 -halfWidthSpace;... % bottom right
        -markerLength/2 -halfWidthSpace]; % bottom left
    % Compute transformation matrix
    % refMarkerIndx = ids==refMarkerId;
    % multiple rvec and tvec are stored for average.
    ref_rvec = sum(cell2mat(StoreSomeRvecs),1)./size(StoreSomeRvecs,1);
    ref_tvec = sum(cell2mat(StoreSomeTvecs),1)./size(StoreSomeTvecs,1);
    % cv.Rodrigues takes resultant rvec, tvec to estimate the positions of
    % a marker's four corners in "Camera's world" (x,y,depth).
    refMarkerCornersObjpts = cv.Rodrigues(ref_rvec)*objectPts + ref_tvec.';
    refMarkerCorners = refMarkerCornersObjpts(1:2,:).';
    % ref_marker_corners = reshape(cell2mat(corners{ref_marker_indx}),2,[]).'; % id = 99, [4 x 2]    
    % Figure the perspective transformation between "camera's world" to the real world with anchor referenceMarker.
    A = [[refMarkerCorners(1,:) 1 0 0 0;
        0 0 0 refMarkerCorners(1,:) 1;
        refMarkerCorners(2,:) 1 0 0 0;
        0 0 0 refMarkerCorners(2,:) 1;
        refMarkerCorners(3,:) 1 0 0 0;
        0 0 0 refMarkerCorners(3,:) 1;
        refMarkerCorners(4,:) 1 0 0 0;
        0 0 0 refMarkerCorners(4,:) 1],...
        -reshape(repmat(reshape(desiredCoordinates.',[],1),2,1),[],2).*reshape(repmat(reshape(refMarkerCorners,[],1),1,2).',[],2)];
    b = reshape(desiredCoordinates.',[],1);
    % Perspective transformation matrix
    H_perspective = reshape([(A\b);1],3,3)';
    disp('Perspective transformation matrix has been defined')
    % Obtain initial pose of robots
    robMarkersIds = ids(~refMarkerIndx);
    [~,sort_ids] = sort(robMarkersIds);
    rob_rvecs = rvecs(~refMarkerIndx);
    rob_rvecs = rob_rvecs(sort_ids);
    rob_tvecs = tvecs(~refMarkerIndx);
    rob_tvecs = rob_tvecs(sort_ids);
    % pre-allocate robot pose
    robMarkerCenter = zeros(2,numRobots); % [x;y] x numRobots
    robMarkerOrient = zeros(2,numRobots); % [thetax;thetay] x numRobots
    for ii = 1:numRobots
        robMarkerCornersObjpts = cv.Rodrigues(rob_rvecs{ii})*objectPts + rob_tvecs{ii}.';
        robMarkerCorners = robMarkerCornersObjpts(1:2,:);

        transfRobMarkerCornors = perspectiveTransform(robMarkerCorners,H_perspective); % it takes [2 x 4] matrix
        % Whatever the four points of a marker in real-world, its center is the average of four points.
        robMarkerCenter(:,ii) = sum(transfRobMarkerCornors,2)./4;
        % test marker orientation
        [~,~,robMarkerOrient(:,ii)] = findOrientation(transfRobMarkerCornors);
     end
    robotXY = robMarkerCenter;
    % heading along y axis due to the way we attach marker on robots <<---------------- Attention!
    robotTheta = robMarkerOrient(2,:); % heading along y axis <<---------------- Attention!
    %--------------------------------------------------------------------
    % display robot initial info
    disp('Robot heading (degrees):')
    disp(robotTheta*180/pi)
    % Control gain and Scale factor reduces magnitude of commands
    controlMaxSpeed = 0.5e-2;%0.01;
    controlScaleFactor = 0.5;%0.002;
    % Add path of utilities to control robots
    K4Driver_path = 'D:\ArUco_Tracking\Utilities\KheperaDriver';
    addpath(K4Driver_path)
    % robotIP is ['93';'99'] char array
    % robotIP = kheperaIDs;
    % Initialize the robots
    verbosity = 0;
    disp('Establishing connections with the robots')
    K4Drv = KheperaDriverInit(kheperaIDs,[],verbosity);
    % Define robot object
    r = Robotarium('NumberOfRobots', numRobots, 'ShowFigure', true,'InitialConditions',[robotXY;robotTheta]);
end
% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 1e7;
% Ordered id's for robots
indxRobots = 1:numRobots;
% Define the robot diameter
robotDiameter = r.robot_diameter;
%%% Control Parameters and Utilities
% Safety distance (barrier ceritificates)
barrierSafetyDist = 1.5*robotDiameter;
% Gain for coverage control law
controlGain = 6;
% Gain for single integrator to unicycle conversion
si2uniGain = 10;
% Set actuator limits
angularVelLimit = 2*pi;%2*pi originally;
% linearVelLimit = 0.49;
linearVelLimit = 0.3/2;%0.6*controlMaxSpeed; % 0.3 ORIGINAL   <---------------------Edit This!
% Geneterate a go-to-goal controller utility
positionController = create_si_position_controller;
% Generate a barrier certificate safety wrap around controller utility
barrierCertificate = create_si_barrier_certificate('SafetyRadius', barrierSafetyDist);
% barrierCertificate = @(x,y) x;
% Generate a single integrator model to unicycle model control conversion
si2uni = ...
    create_si_to_uni_mapping('ProjectionDistance', 1*robotDiameter);
...create_si_to_uni_mapping2('LinearVelocityGain', si2uniGain, 'AngularVelocityLimit', pi/4);
% Area integral grids
[x_mesh,y_mesh] = meshgrid(linspace(0,1,50));
locCostIntegrand = @(q1,q2,p1,p2) bsxfun(@minus,q1,p1).^2+bsxfun(@minus,q2,p2).^2;
%% Define the subdomain function
% Define the time-varying coverage subdomain
domainBoundaries = r.boundaries;
domLength = (domainBoundaries(2) - domainBoundaries(1))/2;
domHeight = (domainBoundaries(4) - domainBoundaries(3))/2;
% Obstacle and Goal Definition
% Obstacle1 = [domainBoundaries(1) domainBoundaries(1) -0.5;
%             0.25 domainBoundaries(4) domainBoundaries(4)];
Obstacle1 = [domainBoundaries(1) domainBoundaries(1) -0;
            -0.1 domainBoundaries(4) domainBoundaries(4)];
% Obstacle2 = [-0.325 -0.325 domainBoundaries(2)-1.2*[1 1] 0.3250 0.3250;
%             domainBoundaries(3) 0.05  0.05  -0.5 -0.5 domainBoundaries(3)];
Obstacle2 = [-0.2 -0.2  0.2 0.2;
            domainBoundaries(3) -0.6  -0.6 domainBoundaries(3)];
Obstacle3 = [domainBoundaries([2 1 1 2 2])*1.1, nan, domainBoundaries([2 2 1 1 2]);
             domainBoundaries([4 4 3 3 4])*1.1, nan, domainBoundaries([4 3 3 4 4])];
% Goal1 =  [-2.5,-2.5,-1.5,-1.5;-1.7,-0.7,-0.7,-1.7];
Goal1 =  [[-2.3,-2.3,-1.5,-1.5]+1.2;-1.7,-0.9,-0.9,-1.7]; % left
Goal2 = [[-2.3,-2.3,-1.5,-1.5]+2.6;-1.7,-0.9,-0.9,-1.7]; % right
% Define subdomain handle
domRadiusX0 = abs(Goal2(1,3)-Goal2(1,2))/2; % domLength/2; %domHeight/2;
domRadiusY0 = abs(Goal2(2,2)-Goal2(2,1))/2; % domHeight/2; %domLength/2;
domainControlGain = 5/diff(domainBoundaries(1:2));
domainMaxSpeed = 9/10/2.2*linearVelLimit/controlScaleFactor;   %  <-----------------------Edit this!!
domainCentroid = sum(Goal2,2)./4; % [0.75;0.75];
referenceCursor = domainCentroid;
% domainCentroidVel = @(x,r) saturate_speed(domainControlGain*(r-x),domainMaxSpeed);
[domainVel,collision_flag] = domainCentroidVel2(domainCentroid,referenceCursor,domainMaxSpeed,domainControlGain,{Obstacle1,Obstacle2,Obstacle3},[domRadiusX0,domRadiusY0]);
hSubdom = @(x) [x(1) + domRadiusX0*[-1 1], x(2) + domRadiusY0*[-1 1]];
hDSubdom = @(v) [v(1)*[1 1],v(2)*[1 1]];
subdomInit = hSubdom(domainCentroid);
% Define the initial configuration on subdomain
posXY0 = [...
    (diff(subdomInit(1:2))/2-robotDiameter)*(2*rand(1,numRobots)-1)+sum(subdomInit(1:2))/2;
    (diff(subdomInit(3:4))/2-robotDiameter)*(2*rand(1,numRobots)-1)+sum(subdomInit(3:4))/2];
%% Trial to Open an UDP object
% ip_thisPC = '10.104.32.36';
% port_thisPC = 9696;
% ip_thatPC = '192.168.1.38';
% port_thatPC = 9191;
% 
% udpobject_thisPC = udp(ip_thatPC,port_thatPC,'LocalPort',port_thisPC);
% fopen(udpobject_thisPC);

%% Visualization Elements
% Construct an extended display object
Fig = r.figure_handle;
%set(Fig,'Position',[500 56 1200 860],'MenuBar','none');
% set(Fig,'Units','normalized','Position',[1 1/3 2/3 2/3]);
% set(Fig,'OuterPosition',[-1920 0 1920 1080]);
% Recover handle for the robots to bring them to the top of the
% visualization stack later
% hRobots = r.figure_handle.Children.Children(1:end-1);
hRobots = gobjects(numRobots,1);
for ii = 1:numRobots
    hRobots(ii) = r.robot_handle{ii};
end
% Set the window to full screen
% set(hFig,'Position',[1921 41 1920 963]);  %[1 41 1536 748.8]);%
% Plot the domain
% hSubdomPlot = rectangle('Position',[subdomInit([1 3]) subdomInit([2 4])-subdomInit([1 3])],'EdgeColor','k','LineWidth',2);
hSubdomPlot = patch('XData',subdomInit([1 1 2 2]),'YData',subdomInit([3 4 4 3]),'EdgeColor','k','LineWidth',2,'FaceColor','none');
% hObstacle1 = rectangle('Position',[-2.55 0.4 2 1.4],'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2);
% hObstacle2 = rectangle('Position',[-0.325 -1.8 0.65 1.4],'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2);
% hObstacle3 = rectangle('Position',[1 0.45 0.65 1.4],'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2);
hObstacle1 = patch('XData',Obstacle1(1,:),'YData',Obstacle1(2,:),'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.75);
hObstacle2 = patch('XData',Obstacle2(1,:),'YData',Obstacle2(2,:),'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.75);
% hInitialbox = rectangle('Position',[-2.3317 -1.2954 1.5545 2.5908],'FaceColor',[0.25 0.5 1],'EdgeColor','k','LineWidth',2);
% hFinalbox = rectangle('Position',[1.6283 -1.6954 0.5945 3.3908],'FaceColor',[0 1 0.5],'EdgeColor','k','LineWidth',2);
hBlueBox = patch('XData',Goal1(1,:),'YData',Goal1(2,:),'FaceColor',[0.25 0.5 1],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.2);
hGreenBox = patch('XData',Goal2(1,:),'YData',Goal2(2,:),'FaceColor',[0 1 0.5],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.2);
% hLinePhi0 = plot(domCenterX+domLength/2*[-1  1],phi0+domCenterY+0.05*domHeight*[0 0],'r','LineWidth',1);
% hLinePhiMag = plot(domCenterX+domLength/2*[-1 1],phiMag+domCenterY+0.05*domHeight*[0 0],'b','LineWidth',1);
% Plot initial positions in 1-d domain
% hInitPos = plot(posXY0(1,:),posXY0(2,:),'.','MarkerSize',30,'LineWidth',3);
hRefCursor = plot(referenceCursor(1),referenceCursor(2),'rx','LineWidth',2);
hDomainCentroid = plot(domainCentroid(1),domainCentroid(2),'ro','LineWidth',2);
hRefCursor2DomCentroid = plot(referenceCursor(1)-domainCentroid(1),referenceCursor(2)-domainCentroid(2),'r--','LineWidth',2);
% Plot tessellation as patches of area under the density curve
colorMat = 0.75*hsv(numRobots);
hVoronoi = gobjects(2,numRobots);
for ii = 1:numRobots
    hVoronoi(:,ii) = patch('XData',nan,'YData',nan,'FaceColor',colorMat(ii,:),'FaceAlpha',0.25);
end
% Plot the density curve
% hDensity = plot(domLineX,phiMag*hPhi((domLineX-domLineX(1))/domLength)+domCenterY,'Color',0.5*[1 1 1],'LineWidth',2);
% Plot the center of mass for each cell (target location)
hCM = plot(nan,nan,'x','MarkerSize',10,'LineWidth',3);
% Bring robot visualization to top of ui stack
uistack(hRobots,'top')

% drawnow;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if metric_plot_flag
hFigLocCost = figure('Position',[769.8 41.8 766.4 328.8]);
yyaxis left
hAggregateError = plot(nan,nan,'LineWidth',2);
grid minor
xlabel('$$t$$ [s]','FontSize',14,'Interpreter','latex')
ylabel('$$\|p-c\|$$ [m]','FontSize',14,'Interpreter','latex')
hLocAxes = gca;
xlim(hLocAxes,[0 100])
yyaxis right
hLocCost = plot(nan,nan,'LineWidth',2);
ylabel('$$\mathcal{H}(p,t)$$/Total Area [m/m]','FontSize',14,'Interpreter','latex')
% if debug_dcdt
%     figure(hFig)
%     hDebugDcdt = extDisp.quiver(nan,nan,nan,nan,0,'LineWidth',3);
% end
end

%%
hFigStopLoop = stopLoop('TERMINATE THE DEMO');

%% TUIO Variables
% Allocate memory
% Number of reference cursors stored
nMemory = 1;
% Available and transitioning slot ID hardcoded constants values
AVAILABLE = 200; TRANSITIONING = 500;
% Id number of currently plotted objects
currentIds = AVAILABLE*ones(1,nMemory);
 
%% Java Libraries
% Add the java libraries to the dynamic class path if not already on it
if ~any(strcmp([pwd,'\javaosc.jar'],javaclasspath('-dynamic')))
    disp('Adding ''javaosc.jar'' to class path...')
    % Library for OSC message handling. (TUIO messages use OSC messages)
    javaaddpath javaosc.jar
end
if ~any(strcmp([pwd,'\yajtl.jar'],javaclasspath('-dynamic')))
    disp('Adding ''yajtl.jar'' to class path...')
    % Library for creating TUIO client/servers and even handling. (Cursors)
    javaaddpath yajtl.jar
end
if ~any(strcmp([pwd,'\DynamicDensityTUIO.jar'],javaclasspath('-dynamic')))
    disp('Adding ''DynamicDensityTUIO.jar'' to class path...')
    % Library for running event listener on separate thread with public
    % methods for getting cursor information (e.g., (x,y), id, start time)
    javaaddpath DynamicDensityTUIO.jar
end
% Create a new TUIO thread to listen for TUIO events (e.g., new cursor)
javaThreadObject = javaObject('dynamic.density.tuio.NewTuioThread');
% Start a new thread
newThread = javaMethod('startTh', javaThreadObject);

%% Loop
% Set flag to demo phases
%       -Phase 1: go to initial position in 1-d domain (LED: red)
%       -Phase 2: set the heading of the robot to the right (LED: yellow)
%       -Phase 3: run coverage control on 1-d domain (LED: green)
%       -Phase 4: display sucess metrics (TODO)
algorithmPhase = 1;
tPhase1 = 1;
% Set LED for phase 1 (red)
% r.set_left_leds(1:numRobots,[255;0;0]*ones(1,numRobots));
% r.set_right_leds(1:numRobots,[255;0;0]*ones(1,numRobots));
% Allocate position matrix and vector of locational cost
if metric_plot_flag
positions = nan(2,numRobots,iterations);
aggregateError = nan(iterations, 1);
VoronoiCellInfoOverTime = cell(numRobots,iterations);
centroidsOverTime = nan(2,numRobots, iterations);
locCost = nan(iterations, 1);
timeVector = nan(iterations, 1);
end
% set(Fig,'OuterPosition',[-1 0 1 1]);

% set(Fig,'units','normalized','OuterPosition',[-1 0 1 1]);
figure(2)
%set(Fig, 'defaultFigureWindowState', 'maximized');
set(Fig,'OuterPosition',[-1920 0 1920 1080]);
% set(gca,'ydir','reversed')
% Voronoi tessellation parameters
powerDiagramWeights = zeros(numRobots, 1); % For power diagram (zeros -> Standard Voronoi cells)
% Iterate for the previously specified number of iterations
for t = 1:iterations
     if hFigStopLoop.Stop()
        break
    end
    
    if simulate_flag
        % Retrieve the most recent poses from the Robotarium.  The time delay is
        % approximately 0.033 seconds
        x = r.get_poses();
        % Rename for convenience
        robotXY = x(1:2,:);
        robotTheta = x(3,:);
    else
        %-----------------------------------------------------------------
        % Grab New Pose Data
        totalIterations = 0;
        LoopFlag1 = true;
        while LoopFlag1
            % grab frame
            img = vid.read();
            if isempty(img)
                break;
            end
            % detect markers and estimate pose
            [corners, ids, rejected] = cv.detectMarkers(img, dictionary, ...
                'DetectorParameters',detectorParams);
        
            if  ~isempty(ids)
                [rvecs, tvecs] = cv.estimatePoseSingleMarkers(corners, ...
                    markerLength, camMatrix, distCoeffs);
            end
            % draw results
            % if ~isempty(ids)
            %     img = cv.drawDetectedMarkers(img, corners, 'IDs',ids);
            %     for i=1:numel(ids)
            %         img = cv.drawAxis(img, camMatrix, distCoeffs, ...
            %             rvecs{i}, tvecs{i}, markerLength*2);
            %     end
            % end
            % if showRejected && ~isempty(rejected)
            %     img = cv.drawDetectedMarkers(img, rejected, 'BorderColor',[255 0 100]);
            % end
            % 
            % if isempty(hImg)
            %     hImg = imshow(img);
            % elseif ishghandle(hImg)
            %     set(hImg, 'CData',img);
            % else
            %     break;
            % end
            % drawnow limitrate;
            % pause(waitTime);
            totalIterations = totalIterations + 1;
            if totalIterations >= 0 && length(ids) == numRobots+1
                LoopFlag1 = false;
                % Obtain pose of robots
                refMarkerIndx = ids==refMarkerId;
                robMarkersIds = ids(~refMarkerIndx);
                [~,sort_ids] = sort(robMarkersIds);
                rob_rvecs = rvecs(~refMarkerIndx);
                rob_rvecs = rob_rvecs(sort_ids);
                rob_tvecs = tvecs(~refMarkerIndx);
                rob_tvecs = rob_tvecs(sort_ids);
                % pre-allocate robot pose
                robMarkerCenter = zeros(2,numRobots); % [x;y] x numRobots
                robMarkerOrient = zeros(2,numRobots); % [thetax;thetay] x numRobots
                for ii = 1:numRobots
                    robMarkerCornersObjpts = cv.Rodrigues(rob_rvecs{ii})*objectPts + rob_tvecs{ii}.';
                    robMarkerCorners = robMarkerCornersObjpts(1:2,:);
            
                    transfRobMarkerCornors = perspectiveTransform(robMarkerCorners,H_perspective); % it takes [2 x 4] matrix
            
                    robMarkerCenter(:,ii) = sum(transfRobMarkerCornors,2)./4;
                    % test marker orientation
                    [~,~,robMarkerOrient(:,ii)] = findOrientation(transfRobMarkerCornors);
                 end
                robotXY = robMarkerCenter;
                robotTheta = robMarkerOrient(2,:); % heading along y axis <<---------------- Attention!
            end
        end
        %----------------------------------------------------------------
        % Set the updated pose for the robots and visualize
        r.set_poses([robotXY;robotTheta]); 
        r.draw_robots();
    end
    % Go through algorithm phases
    if algorithmPhase == 1 % Go to initial positions
        % Go to goal
        dxi = controlScaleFactor*positionController(robotXY,posXY0);
        % Add collision avoidance
        dxi = barrierCertificate(dxi,[robotXY;robotTheta]);
        % Convert to unicycle model
        dxu = si2uni(dxi,[robotXY;robotTheta]);
        % Check if they've converged
        robotsDone = hypot(robotXY(1,:)- posXY0(1,:),robotXY(2,:)-posXY0(2,:))<3e-1; 
        % robotsDone = vecnorm(dxi)<5e-1;
        % Set LED's to yellow if they've converged and red if the have not
        % r.set_left_leds(indxRobots(robotsDone),[255;255;0]*ones(1,nnz(robotsDone)));
        % r.set_right_leds(indxRobots(robotsDone),[255;255;0]*ones(1,nnz(robotsDone)));
        % r.set_left_leds(indxRobots(~robotsDone),[255;0;0]*ones(1,nnz(~robotsDone)));
        % r.set_right_leds(indxRobots(~robotsDone),[255;0;0]*ones(1,nnz(~robotsDone)));
        if all(robotsDone)
            % Move to phase 2 if they are all done
            algorithmPhase = 2;
            tPhase2 = t + 1;
            % Define current bounding box
            boundingBox = [...
                subdomInit([1 3]);
                subdomInit([1 4]);
                subdomInit([2 4]);
                subdomInit([2 3])]; % the bounding box in clockwise order
            
            % Get the Voronoi tessellation (from Matlab File Exchange: Fast Voronoi)
            [V,C,polyXY] = power_bounded(robotXY(1,:).',robotXY(2,:).', powerDiagramWeights, boundingBox);
            % Compute Voronoi cell area integrals
            polyV = polyshape(polyXY(1,:),polyXY(2,:),'Simplify',false);
            m = area(polyV,1:numRobots);
            m = abs(m);
            c = zeros(2,numRobots);
            [c(1,:),c(2,:)] = centroid(polyV,1:numRobots);
            set(hCM,'XData',c(1,:),'YData',c(2,:))
            for ii = 1:numRobots
                set(hVoronoi(:,ii),'XData',V(C{ii},1),'YData',V(C{ii},2))
            end
            % Start a clock for the domain function
            timer = tic;
            timerNewTime = toc(timer);
            drawnow
        end
    elseif algorithmPhase == 2 % Coverage control
        % Update TUIO cursor information
        newCursorIds = javaThreadObject.getId()';
        if any(newCursorIds>=AVAILABLE)
            disp('Excedeed maximum ID allocation, please reset TUIO client app...')
        end
        % Positions
        newCursorX =  (domainBoundaries(1)+(domainBoundaries(2)-domainBoundaries(1)).*javaThreadObject.getX()');
        newCursorY =  (domainBoundaries(4)-(domainBoundaries(4)-domainBoundaries(3)).*javaThreadObject.getY()');
        %% Implement TUIO cursor logic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Check if a new cursor was added.
        unmatchedCursorIds = setdiff(newCursorIds,currentIds);
        if ~isempty(unmatchedCursorIds)
            % There are unaccounted for cursors being broadcasted, store
            % them in a new vector for ease of handling.
            availableIdSlots = ...
                find((currentIds>=AVAILABLE)&(currentIds<TRANSITIONING));
            % Assign all unmatched cursors
            if length(availableIdSlots)<length(unmatchedCursorIds)
                disp(['More cursors active than Gaussians available. ',...
                    'Discarding some...'])
            end
            maxIter = min(length(unmatchedCursorIds),length(availableIdSlots));
            for iAdd = 1:maxIter
                % Cursor not accounted for. Add to pool.
                currentIds(availableIdSlots(iAdd)) = unmatchedCursorIds(iAdd);
                referenceCursor(:,availableIdSlots(iAdd)) = [...
                    newCursorX(iAdd);
                    newCursorY(iAdd)];
            end
        end
        %
        %------------------------------------------------------------------
        % Check if a TUIO cursor is transtitioning out of the pool
        if any(currentIds<AVAILABLE)
            % Determine if any and which TUIO cursors are obsolete
            oldCursorIds = setdiff(currentIds,...
                [newCursorIds,AVAILABLE,TRANSITIONING]);
            [~,transitionIndices] = ismember(oldCursorIds,currentIds);
            if ~isempty(oldCursorIds)
                % keyboard
                for kTransition = 1:length(transitionIndices)
                    % Change ID to transitioning and removal transition
                    % time, if transition set to final time (not already
                    % transitioning)
                    currentIds(transitionIndices(kTransition)) = TRANSITIONING;
                end
            end
        end
        %
        %------------------------------------------------------------------
        % Check if any transitioning TUIO cursor needs to be removed
        if any(currentIds==TRANSITIONING)
            % keyboard
            % If transition more than few seconds, it doesn't contribute,
            % remove it from the pool of Gaussians and set to final time
            transitioningIds = (currentIds==TRANSITIONING);
            currentIds(transitioningIds) = AVAILABLE;
        end
        %
        %------------------------------------------------------------------
        % Change reference centroid of all active Gaussians
        if any(currentIds<AVAILABLE)
            % Get the matching Ids from the current pool of Gaussians
            [matchedIds,kMove] = ismember(currentIds,newCursorIds);
            % Sanity check
            if length(matchedIds(matchedIds==1))~=length(newCursorIds)
                disp(['Something wrong in the TUIO logic! ',...
                    'Did not trim/add the right Ids'])
            end
            % Update the position of the centroids
            for iMove = 1:nMemory
                if kMove(iMove) ~= 0 ...
                        && iMove<=numel(newCursorX) && iMove<=numel(newCursorY)
                    referenceCursor(:,iMove) = [...
                        newCursorX(kMove(iMove));
                        newCursorY(kMove(iMove))];
                end
                % Check if we must continue this loop
                if all(kMove(iMove+1:end)==0)
                    break
                end
            end
        end
        %% End TUIO cursor logic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Timer
        tau = (t-tPhase2+1);
        timerPrevTime = timerNewTime;
        timerNewTime = toc(timer);
        % Update the subdomain position
        subDom = hSubdom(domainCentroid);
        % send this info
        % fprintf(udpobject_thisPC,subDom);
        % subdomProjector = projectiveTransform([subDom([1 1 2 2]);subDom([3 4 4 3])],H);
        % domainVel = domainCentroidVel(domainCentroid,referenceCursor);
        [domainVel,collision_flag] = domainCentroidVel2(domainCentroid,referenceCursor,domainMaxSpeed,domainControlGain,{Obstacle1,Obstacle2,Obstacle3},[domRadiusX0,domRadiusY0]);
        dSubDom = hDSubdom(domainVel);
        % Update subdomain visualization
        if collision_flag
            set(hSubdomPlot,'XData',subDom([1 1 2 2]),'YData',subDom([3 4 4 3]),'EdgeColor','r','LineWidth',4)
            % extDisp.set(hSubdomPlotProjector,'XData',subdomProjector(1,:),'YData',subdomProjector(2,:),'EdgeColor','r')
        else
            set(hSubdomPlot,'XData',subDom([1 1 2 2]),'YData',subDom([3 4 4 3]),'EdgeColor','k','LineWidth',2)
            % extDisp.set(hSubdomPlotProjector,'XData',subdomProjector(1,:),'YData',subdomProjector(2,:),'EdgeColor','k')
        end
        if simulate_flag
            dt = r.time_step;
            % timeVector(tau) = timerNewTime;
        else
            dt = timerNewTime - timerPrevTime;
            % timeVector(tau) = timerNewTime;
        end
        set(hDomainCentroid,'XData',domainCentroid(1),'YData',domainCentroid(2))
        set(hRefCursor,'XData',referenceCursor(1),'YData',referenceCursor(2))
        set(hRefCursor2DomCentroid,'XData',[referenceCursor(1) domainCentroid(1)],'YData',[referenceCursor(2) domainCentroid(2)])
        
        % domainCentroidProjector = projectiveTransform(domainCentroid(:),H);
        % referenceCursorProjector = projectiveTransform(referenceCursor(:),H);
        % set(hDomainCentroidProjector,'XData',domainCentroidProjector(1),'YData',domainCentroidProjector(2))
        % set(hRefCursorProjector,'XData',referenceCursorProjector(1),'YData',referenceCursorProjector(2))
        % set(hRefCursor2DomCentroidProjector,'XData',[referenceCursorProjector(1) domainCentroidProjector(1)],'YData',[referenceCursorProjector(2) domainCentroidProjector(2)])
        % Integrate position numerically
        domainCentroid = domainCentroid + dt*domainVel;
        % Store the positions at the current time
        % positions(:,:,tau) = robotXY;
        % Compute the Voronoi tessellation
        boundingBox = [...
            subDom([1 3]);
            subDom([1 4]);
            subDom([2 4]);
            subDom([2 3])]; % the bounding box in clockwise order
        % Get the Voronoi tessellation (from Matlab File Exchange: Fast Voronoi)
        [V,C,polyXY,AdjacencyXY] = power_bounded(robotXY(1,:).',robotXY(2,:).', powerDiagramWeights, boundingBox);
        % Get the Voronoi cells as polyshapes
        polyV = polyshape(polyXY(1,:),polyXY(2,:),'Simplify',false);
        % Compute Voronoi cell area integrals using polyshape
        m = area(polyV,1:numRobots);
        m = abs(m);
        c = zeros(2,numRobots);
        [c(1,:),c(2,:)] = centroid(polyV,1:numRobots);
        % Determine adjacency and find the neighbor boundary vertex
        % indices. Also determine Voronoi cells in contact with the
        % subdomain and find the vertex indices
        [adjacency, iDVij, boundCells, iDSi] = voronoiAdjacency(AdjacencyXY ,subDom);
        % Allocate partial derivatives, normal array, and cost variables
        dcdt = zeros(2,numRobots);
        dcdp = zeros(2*numRobots);
        normalVectors = zeros(2,numRobots,numRobots);
        interiorCellBoundary = zeros(2,numRobots,numRobots);

        if metric_plot_flag
        aggregateError(tau) = 0;
        locCost(tau) = 0;
        VoronoiCellInfoOverTime(:,tau) = AdjacencyXY;
        centroidsOverTime(:,:,tau) = c;
        end

        for ii = 1:numRobots
            % Relabel for readability
            pii = robotXY(:,ii);
            cii = c(:,ii);
            mii = m(ii);
            if ii < numRobots
                for jj = ii+1:numRobots
                    if adjacency(ii,jj) ~= 0
                        % Relabel for readability
                        pjj = robotXY(:,jj);
                        cjj = c(:,jj);
                        mjj = m(jj);
                        normpij = norm(pii-pjj);
                        normalOutward = (pjj-pii)/normpij;
                        normalVectors(:,jj,ii) =  normalOutward; % i's Neighbor j
                        normalVectors(:,ii,jj) = -normalOutward; % j's Neighbor i
                        Nij = normalOutward*normalOutward.';
                        % Get the vertices for the boundary
                        V1 = AdjacencyXY{ii}(iDVij(ii,jj,1),:).';
                        V2 = AdjacencyXY{ii}(iDVij(ii,jj,2),:).';
                        V2_minus_V1 = V2 - V1;
                        interiorCellBoundary(:,jj,ii) = V2_minus_V1;
                        interiorCellBoundary(:,ii,jj) = V2_minus_V1;
                        normV1V2 = norm(V2_minus_V1);
                        % for dcidpj
                        V1_minus_cii = V1 - cii;
                        V1_minus_pjj = V1 - pjj;
                        % for dcjdpi
                        V1_minus_cjj = V1 - cjj;
                        V1_minus_pii = V1 - pii;
                        % dcidpj
                        dcidpj = -normV1V2/(mii*normpij)*...
                            (V1_minus_cii*V1_minus_pjj.'   ...
                            +0.5*(V1_minus_cii*V2_minus_V1.'+...
                            V2_minus_V1*V1_minus_pjj.') ...
                            +1/3*(V2_minus_V1*V2_minus_V1.') ...
                            );
                        % Specular reflection
                        dcidpi = dcidpj*(2*Nij-eye(2));
                        % Store
                        dcdp(2*ii-1:2*ii,2*jj-1:2*jj) = dcidpj;
                        dcdp(2*ii-1:2*ii,2*ii-1:2*ii) = ...
                            dcdp(2*ii-1:2*ii,2*ii-1:2*ii) + dcidpi;
                        % dcjdpi
                        dcjdpi = -normV1V2/(mjj*normpij)*...
                            (V1_minus_cjj*V1_minus_pii.'   ...
                            +0.5*(V1_minus_cjj*V2_minus_V1.'+...
                            V2_minus_V1*V1_minus_pii.') ...
                            +1/3*(V2_minus_V1*V2_minus_V1.') ...
                            );
                        % Specular reflection
                        dcjdpj = dcjdpi*(2*Nij-eye(2));
                        % Store
                        dcdp(2*jj-1:2*jj,2*ii-1:2*ii) = dcjdpi;
                        dcdp(2*jj-1:2*jj,2*jj-1:2*jj) = ...
                            dcdp(2*jj-1:2*jj,2*jj-1:2*jj) + dcjdpj;
                    end
                end
            end
            % Compute dcidt
            if boundCells(ii) > 0
                jj = find(iDSi{ii},1,'first');
                while jj <= length(iDSi{ii})
                    if iDSi{ii}(jj) ~= 0
                        V1 = AdjacencyXY{ii}(jj,:).';
                        if jj < length(iDSi{ii})
                            if iDSi{ii}(jj+1) ~=0
                                V2 = AdjacencyXY{ii}(jj+1,:).';
                            else
                                jj = jj + 1;
                                continue
                            end
                        else
                            if iDSi{ii}(1) ~=0
                                V2 = AdjacencyXY{ii}(1,:).';
                            else
                                jj = jj + 1;
                                continue
                            end
                        end
                        % Vertices are provided in cw orientation. Use
                        % vector orientation to determine which boundary
                        % derivative to use, within boundAngTol degrees
                        boundAngTol = 1;
                        exteriorBoundary = V2-V1;
                        normExteriorBoundary = norm(exteriorBoundary);
                        boundaryDir = exteriorBoundary/normExteriorBoundary;
                        boundAngle = atan2d(boundaryDir(2),boundaryDir(1));
                        interiorV1s = AdjacencyXY{ii}(nonzeros(iDVij(ii,:,1)),:);
                        interiorV2s = AdjacencyXY{ii}(nonzeros(iDVij(ii,:,2)),:);
                        matchingBoundary = ...
                            (ismembertol(interiorV1s,V1','ByRows',true)&ismembertol(interiorV2s,V2','ByRows',true))...
                            |(ismembertol(interiorV1s,V2','ByRows',true)&ismembertol(interiorV2s,V1','ByRows',true));
                        % Check that the boudary vertices dont belong to an
                        % interior dVij
                        if ~any(matchingBoundary)
                            % Apply unit outward normal to derivative
                            if abs(boundAngle-90) < boundAngTol % Left boundary
                                dqdt = -dSubDom(1);
                            elseif abs(boundAngle+90) < boundAngTol % Right boundary
                                dqdt = dSubDom(2);
                            elseif abs(boundAngle-180) < boundAngTol || abs(boundAngle+180) < boundAngTol % Bottom boundary
                                dqdt = -dSubDom(3);
                            elseif abs(boundAngle) < boundAngTol % Top boundary
                                dqdt = dSubDom(4);
                            else
                                dqdt = 0;
                                warning('Voronoi cell ''Subdomain'' boundary not aligned with subdomain')
                            end
                        else % Boundary aligned with interior boundary
                            dqdt = 0;
                        end
                        %
                        dcidt_analy = ...
                            0.5*dqdt/mii*((V1-cii)+(V2-cii))*norm(V1-V2);
                        % int (q-ci) dq/dt dq / mi
                        dcdt(:,ii) = dcdt(:,ii) + dcidt_analy;%
                        
                    end
                    jj = jj + 1;
                end
            end
            if metric_plot_flag
            % Integrate for locational cost
            aggregateError(tau) = norm(robotXY(:)-c(:));% + trapz(q_line, abs(q_line - p(ii)).*phi_line);
            % 
            locCost(tau) = locCost(tau) + polygonIntegral(@(x,y)locCostIntegrand(x,y,pii(1),pii(2)),AdjacencyXY{ii},x_mesh,y_mesh)/(diff(subDom(1:2))*diff(subDom(3:4)));% + trapz(q_line, abs(q_line - p(ii)).*phi_line);
            if tau*r.time_step < 100
                tauIndLoc = (1:tau);
                tauLocCost = timeVector(tauIndLoc);%tauIndLoc*r.time_step;
                set(hAggregateError, 'XData', tauLocCost, 'YData',aggregateError(tauIndLoc));
                set(hLocCost, 'XData', tauLocCost, 'YData',locCost(tauIndLoc));
            else
                tauIndLoc = (tau-floor(100/r.time_step):tau);
                tauLocCost = timeVector(tauIndLoc);%tauIndLoc*r.time_step;
                xlim(hLocAxes,tauLocCost([1 end]))
                set(hAggregateError, 'XData', tauLocCost, 'YData',aggregateError(tauIndLoc));
                set(hLocCost, 'XData', tauLocCost, 'YData',locCost(tauIndLoc));
            end
            end
            % Update Voronoi tessellation visualization
            set(hVoronoi(:,ii),'XData',V(C{ii},1),'YData',V(C{ii},2))
            % VProjector = projectiveTransform([V(C{ii},1).';V(C{ii},2).'],H);
            % set(hVoronoiProjector(ii),'XData',VProjector(1,:),'YData',VProjector(2,:))
        end
        % Update center of mass visualization
        set(hCM,'XData',c(1,:),'YData',c(2,:))
        % CMProjector = projectiveTransform(c,H);
        % set(hCMProjector,'XData',CMProjector(1,:),'YData',CMProjector(2,:))
        % Compute coverage control law (note: it's transposed)
        % dp = ((eye(2*numRobots)))*(controlGain*(c(:)-robotXY(:))+dcdt(:));%);
        dp = ((eye(2*numRobots)+dcdp))*(controlGain*(c(:)-robotXY(:))+dcdt(:));
        % dp = ((eye(2*numRobots)+dcdp))*(controlGain*(c(:)-robotXY(:)));%);
        % dp = ((eye(2*numRobots)+dcdp+dcdp^2))*(controlGain*(c(:)-robotXY(:))+dcdt(:));%);
        % dp = ((eye(2*numRobots)+dcdp+dcdp^2+dcdp^3))*(controlGain*(c(:)-robotXY(:))+dcdt(:));%);
        % dp = ((eye(2*numRobots)+dcdp+dcdp^2+dcdp^3+dcdp^4+dcdp^5+dcdp^6))*(controlGain*(c(:)-robotXY(:))+dcdt(:));%);
        % dp = ((eye(2*numRobots)-dcdp))\(controlGain*(c(:)-robotXY(:)));
        % dp = (eye(2*numRobots)-dcdp)\(controlGain*(c(:)-robotXY(:))+dcdt(:));
        dxi = controlScaleFactor*reshape(dp,2,[]);
        % Scale and Stack control commands to be sent
        dxu = si2uni(dxi,[robotXY;robotTheta]);
        % Set LEDs to green if they've converged and yellow if the have not
        robotsDone = hypot(c(1,:)-robotXY(1,:),c(2,:)-robotXY(2,:))<1e-2;
        % r.set_left_leds(indxRobots(robotsDone),[0;255;0]*ones(1,nnz(robotsDone)));
        % r.set_right_leds(indxRobots(robotsDone),[0;255;0]*ones(1,nnz(robotsDone)));
        % r.set_left_leds(indxRobots(~robotsDone),[255;255;0]*ones(1,nnz(~robotsDone)));
        % r.set_right_leds(indxRobots(~robotsDone),[255;255;0]*ones(1,nnz(~robotsDone)));
    end
    %% Send velocities to agents
    % Saturate the control to avoid actuator limits
    dxu(1,dxu(1,:)>linearVelLimit) = linearVelLimit;
    dxu(1,dxu(1,:)<-linearVelLimit) = -linearVelLimit;
    dxu(2,dxu(2,:)>angularVelLimit) = angularVelLimit;
    dxu(2,dxu(2,:)<-angularVelLimit) = -angularVelLimit;
    if simulate_flag
        % Set velocities of agents 1,...,N
        r.set_velocities(1:numRobots, dxu);
        % Send the previously set velocities to the agents.  This function must be called!
        r.step();
    else
        % Send velocity updates to robots
        KheperaSetSpeed(K4Drv, dxu(1,:), dxu(2,:))
        % KheperaSetSpeed(K4Drv,0,0)
    end
 
end
%% Terminate
javaMethod('stopTh', javaThreadObject,newThread)
clear javaThreadObject newThread
% Remove libraries from dynamic path
if any(strcmp([pwd,'\javaosc.jar'],javaclasspath('-dynamic')))
    disp('Removing ''javaosc.jar'' from class path...')
    javarmpath javaosc.jar
end
if any(strcmp([pwd,'\yajtl.jar'],javaclasspath('-dynamic')))
    disp('Removing ''yajtl.jar'' from class path...')
    javarmpath yajtl.jar
end
if any(strcmp([pwd,'\DynamicDensityTUIO.jar'],javaclasspath('-dynamic')))
    disp('Removing ''DynamicDensityTUIO.jar'' from class path...')
    javarmpath DynamicDensityTUIO.jar
end
if ~simulate_flag
    % Stop all robots at the end
    KheperaSetSpeed(K4Drv, 0, 0)
    % Disconnect the Khepera driver client
    KheperaDriverDisconnect(K4Drv)
    % Terminate the camera object
    vid.release()
    % Close the "stop" window it is still open

    % fullpathdir = [pwd,'\data\'];
    % filename = unique_filename([fullpathdir,domainType,'Coverage_4Robots']);
    % filenameFig1 = unique_filename_fig([fullpathdir,domainType,'Coverage_4Robots_RobotFig']);
    % filenameFig2 = unique_filename_fig([fullpathdir,domainType,'Coverage_4Robots_LocCostFig']);
    % hasBeenRecorded = ~isnan(timeVector);
    % positions = positions(:,:,hasBeenRecorded);
    % VoronoiCellInfoOverTime = VoronoiCellInfoOverTime(:,hasBeenRecorded);
    % centroidsOverTime = centroidsOverTime(:,:,hasBeenRecorded);
    % timeVector = timeVector(hasBeenRecorded);
    % locCost = locCost(hasBeenRecorded);
    % aggregateError = aggregateError(hasBeenRecorded);
    % save(filename,'positions','timeVector','locCost','aggregateError','VoronoiCellInfoOverTime','centroidsOverTime');
    % savefig(hFig,filenameFig1)
    % savefig(hFigLocCost,filenameFig2)
end
% fclose(udpobject_thisPC);
% delete(udpobject_thisPC);
% disp('Removing UDP object...')
% clear ip_thisPC port_thisPC ip_thatPC port_thatPC udpobject_thisPC
hFigStopLoop.Clear();
clear hFigStopLoop;
% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();
pause(0.1);
disp('The Operation Is Completely Terminated!!');