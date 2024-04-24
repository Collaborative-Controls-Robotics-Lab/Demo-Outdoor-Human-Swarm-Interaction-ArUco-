%% Put the reference marker alone to determine the transformation!!!


%
clear all;close all;clc;
%

domainBoundaries = [-1.5 1.5 -1.8288 3-1.8288];
%
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



%%
% reference marker id
refMarkerId = 99;
numRobots = 0;
%
disp('Establishing connections with camera')
%------------------------------------------------------------------
% Add mexopencv functionalities
addpath('C:\dev\mexopencv')
addpath('C:\dev\mexopencv/opencv_contrib');
% load parameters for ArUco locolization
load camera_parameters_1280_720_logi.mat -mat camMatrix distCoeffs % <<----- Edit this!
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
vid = cv.VideoCapture(1); % 0 -- open default camera    <<------------------ Edit this!
vid.FrameWidth = 1280; % Need re-calibration if changed <<------------------ Edit this!
vid.FrameHeight = 720; % <<------------------------------------------------- Edit this!
waitTime = 0.01;  % 10 msec
if ~vid.isOpened()
    error('failed to initialize VideoCapture');
end
%% Visualization
Fig = figure('MenuBar','none','WindowState','fullscreen');
% grab frame
img = vid.read();
hImg = imshow(img);

%5
% Fetch initial localization info for perspective transformation matrix
totalIterations0 = 0;
%hImg = [];
LoopFlag0 = true;
StoreSomeCorners = cell(10,1);
while LoopFlag0
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
    if ~isempty(ids)
        img = cv.drawDetectedMarkers(img, corners, 'IDs',ids);
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
    if length(ids) == numRobots + 1
        StoreSomeCorners{totalIterations0+1} = cell2mat(corners{:});
        totalIterations0 = totalIterations0 + 1;
    end
    if totalIterations0>=10
        StoreSomeCorners{totalIterations0+1} = cell2mat(corners{:});
        LoopFlag0 = false;
    end
end

halfWidthSpace = 1.8;
desiredCoordinates = [-markerLength/2 -halfWidthSpace+markerLength;... % top left
    markerLength/2 -halfWidthSpace+markerLength;... % top right
    markerLength/2 -halfWidthSpace;... % bottom right
    -markerLength/2 -halfWidthSpace]; % bottom left

% Compute transformation matrix
refMarkerIndx = ids==refMarkerId;
% points in local frame
% objectPts = [-markerLength/2    markerLength/2    markerLength/2   -markerLength/2;...
%     markerLength/2    markerLength/2   -markerLength/2   -markerLength/2;...
%     0         0         0         0];
% refMarkerCornersObjpts = cv.Rodrigues(rvecs{refMarkerIndx})*objectPts + tvecs{refMarkerIndx}.';
% refMarkerCorners = refMarkerCornersObjpts(1:2,:).';

refMarkerCorners = reshape(sum(cell2mat(StoreSomeCorners),1)./11,2,[]).';
% refMarkerCorners = reshape(cell2mat(corners{:}),2,[]).';
% ref_marker_corners = reshape(cell2mat(corners{ref_marker_indx}),2,[]).'; % id = 99, [4 x 2]
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
H_inv = inv(H_perspective);
disp('Perspective transformation matrix & its inverse have been defined')
disp('Now implement the augmented reality')
disp('-----------------------------------')
%
AR_Obstacle1 = invPerspectiveTransform(Obstacle1,H_inv);
AR_Obstacle2 = invPerspectiveTransform(Obstacle2,H_inv);
AR_Domain = invPerspectiveTransform(Obstacle3(:,7:10),H_inv);
AR_Goal1 = invPerspectiveTransform(Goal1,H_inv);
AR_Goal2 = invPerspectiveTransform(Goal2,H_inv);
hold on
hAR_Obstacle1 = patch('XData',AR_Obstacle1(1,:),'YData',AR_Obstacle1(2,:),'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.5);
hold on
hAR_Obstacle2 = patch('XData',AR_Obstacle2(1,:),'YData',AR_Obstacle2(2,:),'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.5);
hold on
hAR_Domain = patch('XData',AR_Domain(1,:),'YData',AR_Domain(2,:),'FaceColor','none','EdgeColor','k','LineWidth',4);
hold on
hBlueBox = patch('XData',AR_Goal1(1,:),'YData',AR_Goal1(2,:),'FaceColor',[0.25 0.5 1],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.2);
hold on
hGreenBox = patch('XData',AR_Goal2(1,:),'YData',AR_Goal2(2,:),'FaceColor',[0 1 0.5],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.2);


%%
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
    if ~isempty(ids)
        img = cv.drawDetectedMarkers(img, corners, 'IDs',ids);
        for i=1:numel(ids)
            img = cv.drawAxis(img, camMatrix, distCoeffs, ...
                rvecs{i}, tvecs{i}, markerLength*2);
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
    drawnow limitrate;
    pause(waitTime);
end
%%

vid.release()

