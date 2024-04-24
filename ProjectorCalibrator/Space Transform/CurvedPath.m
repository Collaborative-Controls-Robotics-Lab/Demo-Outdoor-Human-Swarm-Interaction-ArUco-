%% CURVEDPATH Sets up the space to perform braids on a track.
close all
TrackPlot = figure('Name','Braid Track','NumberTitle','off','Position',[11 46 1580 767],'color','w'); % [3201 -179 1920 1003]
grid on
hold on

% Some colors in a cell, e.g. for plotting in loops
grayIntensity = 0.3;
c = {'g','r','m','b','c','k','y'};

% Define the width and height of track in decimeters, convert to meters
OuterBox = [30 15]/10;
InnerGap = 8/10;
plot([0 0 OuterBox(1) OuterBox(1) 0],[0 OuterBox(2) OuterBox(2) 0 0],'-y','linewidth',3)
axis tight
set(gca,'PlotBoxAspectRatio',[1 1 1])
set(gca,'DataAspectRatio',[1 1 1])
title('Track','fontsize',16)
xlabel('East (m)','fontsize',14)
ylabel('North (m)','fontsize',14)
axis manual

%Number of Trapezoids
numTrap = 4;
x = 0:1/numTrap:1;
LeftOuterLoop = [OuterBox(2)/2.*cos(-pi.*x-pi/2)+OuterBox(2)/2;OuterBox(2)/2.*sin(-pi.*x-pi/2)+OuterBox(2)/2];
TopOuterLine = [OuterBox(2)/2*(1-x)+(OuterBox(1)-OuterBox(2)/2)*x;OuterBox(2)*ones(size(x))];
RightOuterLoop = [OuterBox(2)/2.*cos(-pi.*x+pi/2)+(OuterBox(1)-OuterBox(2)/2);OuterBox(2)/2.*sin(-pi.*x+pi/2)+(OuterBox(2)/2)];
BottomOuterLine = [OuterBox(2)/2*x+(OuterBox(1)-OuterBox(2)/2)*(1-x);OuterBox(2)*zeros(size(x))];

OuterTrack = [LeftOuterLoop,TopOuterLine,RightOuterLoop,BottomOuterLine];
OuterTrack = OuterTrack(:,end:-1:1);

LeftInnerLoop = [(InnerGap)/2.*cos(-pi.*x-pi/2)+OuterBox(2)/2;(InnerGap)/2.*sin(-pi.*x-pi/2)+OuterBox(2)/2];
TopInnerLine = [OuterBox(2)/2*(1-x)+(OuterBox(1)-OuterBox(2)/2)*x;(OuterBox(2)/2+InnerGap/2)*ones(size(x))];
RightInnerLoop = [(InnerGap)/2.*cos(-pi.*x+pi/2)+OuterBox(1)-OuterBox(2)/2;(InnerGap)/2.*sin(-pi.*x+pi/2)+OuterBox(2)/2];
BottomInnerLine = [OuterBox(2)/2*x+(OuterBox(1)-OuterBox(2)/2)*(1-x);(OuterBox(2)/2-InnerGap/2)*ones(size(x))];

InnerTrack = [LeftInnerLoop,TopInnerLine,RightInnerLoop,BottomInnerLine];
InnerTrack = InnerTrack(:,end:-1:1);

plot(OuterTrack(1,:),OuterTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1]);
plot(InnerTrack(1,:),InnerTrack(2,:),'linewidth',2,'Color',grayIntensity*[1 1 1]);


%% Defining Braid Parameters
% Number of Agents
N = 5;

% Number of Braids
M_Trapezoids = 4;
M_Rectangles = 20;
M = 2*M_Rectangles+2*numTrap*M_Trapezoids;

BraidPoints = zeros(2,N,M);

% Find the Braid Points by distributing uniformly among rectangles and
% trapezoids. First M_Rectangles braid points belong to a rectangle, then
% numTrap*M_Trapezoids to trapezoids, the M_Rectangles to rectangles and
% numTrap*M_Trapezoids to trapezoids.

for i = 1:M
    if(i<=M_Rectangles)
        for j = 1:M_Rectangles
            % Set the x coordinate constant for braid string step
            BraidPoints(1,:,j) = repmat(...
                BottomOuterLine(1,1)*((j-1)/(M_Rectangles))+BottomOuterLine(1,end)*(1-(j-1)/(M_Rectangles)),...
                N,1);
            % Interpolate between inner and outer boundaries to obtain
            % uniform spread of agents at each step.
            BraidPoints(2,:,j) = (...
                BottomInnerLine(2,1)*(1-(0:N-1)/(N-1))+BottomOuterLine(2,1)*((0:N-1)/(N-1)) ...
                ).';
        end
        
    elseif(i<=M_Rectangles+numTrap*M_Trapezoids)
        for ii = 1:numTrap
            for j = 1:M_Trapezoids%+M_Trapezoids*(ii-1)/(numTrap-1)
                % Uniformly break up the inner and outer boundary along
                % path
                InnerBoundary = RightInnerLoop(:,end-(ii-1))*(1-(j-1)/(M_Trapezoids))+RightInnerLoop(:,end-(ii))*((j-1)/(M_Trapezoids));
                OuterBoundary = RightOuterLoop(:,end-(ii-1))*(1-(j-1)/(M_Trapezoids))+RightOuterLoop(:,end-(ii))*((j-1)/(M_Trapezoids));
                BraidPoints(1,:,j+M_Rectangles+M_Trapezoids*(ii-1)) = ...
                    InnerBoundary(1)*(1-(0:N-1)/(N-1))+OuterBoundary(1)*((0:N-1)/(N-1));%
                % Interpolate between inner and outer boundaries to obtain
                % uniform spread of agents at each step.
                BraidPoints(2,:,j+M_Rectangles+M_Trapezoids*(ii-1)) = (.../(numTrap-1)
                    InnerBoundary(2)*(1-(0:N-1)/(N-1))+OuterBoundary(2)*((0:N-1)/(N-1)) ...
                    ).';
                % keyboard
            end
        end
    elseif(i<=2*M_Rectangles+numTrap*M_Trapezoids)
        for j = 1:M_Rectangles
            % Set the x coordinate constant for braid string step
            BraidPoints(1,:,j+M_Rectangles+numTrap*M_Trapezoids) = repmat(...
                TopOuterLine(1,1)*((j-1)/(M_Rectangles))+TopOuterLine(1,end)*(1-(j-1)/(M_Rectangles)),...
                N,1);
            % Interpolate between inner and outer boundaries to obtain
            % uniform spread of agents at each step.
            BraidPoints(2,:,j+M_Rectangles+numTrap*M_Trapezoids) = (...
                TopInnerLine(2,1)*(1-(0:N-1)/(N-1))+TopOuterLine(2,1)*((0:N-1)/(N-1)) ...
                ).';
        end
    else
        for ii = 1:numTrap
            for j = 1:M_Trapezoids%+M_Trapezoids*(ii-1)/(numTrap-1)
                % Uniformly break up the inner and outer boundary along
                % path
                InnerBoundary = LeftInnerLoop(:,end-(ii-1))*(1-(j-1)/(M_Trapezoids))+LeftInnerLoop(:,end-(ii))*((j-1)/(M_Trapezoids));
                OuterBoundary = LeftOuterLoop(:,end-(ii-1))*(1-(j-1)/(M_Trapezoids))+LeftOuterLoop(:,end-(ii))*((j-1)/(M_Trapezoids));
                BraidPoints(1,:,j+2*M_Rectangles+numTrap*M_Trapezoids+M_Trapezoids*(ii-1)) = ...
                    InnerBoundary(1)*(1-(0:N-1)/(N-1))+OuterBoundary(1)*((0:N-1)/(N-1));%
                % Interpolate between inner and outer boundaries to obtain
                % uniform spread of agents at each step.
                BraidPoints(2,:,j+2*M_Rectangles+numTrap*M_Trapezoids+M_Trapezoids*(ii-1)) = (.../(numTrap-1)
                    InnerBoundary(2)*(1-(0:N-1)/(N-1))+OuterBoundary(2)*((0:N-1)/(N-1)) ...
                    ).';
                % keyboard
            end
        end
    end
end

plot(squeeze(BraidPoints(1,:,:)),squeeze(BraidPoints(2,:,:)),'o','Color',grayIntensity*[1 1 1])


%% The Braid Path

% Generate an N braid string to follow, excluding the trivial generator.
Braid = [randi(N-1,1,M);zeros(1,M)];

% % An M = 80 randomly generated braid string:
% Braid = ...
% [2  4  3  3  4  2  4  4  2  3  1  1  3  4  4  1  3  2  1  2 ...
%  1  4  2  3  1  3  2  3  3  3  2  1  1  4  1  4  3  4  1  2 ...
%  1  4  1  4  4  4  1  2  2  4  2  4  1  2  1  1  4  3  3  1 ... 
%  4  3  2  3  2  1  1  1  1  1  2  1  4  4  2  2  2  4  2  1];

% Compute the node each agent will be at each step of the braid.
BraidAgents = repmat((1:N)',1,M); % Start with the trivial generator
for i = 2:M
    BraidHat = Braid(1,i);
    % Swap agent sigma(i) and sigma(i)+1
    BraidAgents(:,i) = BraidAgents(:,i-1)...
        -(BraidAgents(BraidHat,i-1)*((1:N).'==BraidHat)+BraidAgents(BraidHat+1,i-1)*((1:N).'==BraidHat+1))...
        +(BraidAgents(BraidHat+1,i-1)*((1:N).'==BraidHat)+BraidAgents(BraidHat,i-1)*((1:N).'==BraidHat+1));
    if N>=5
        if(BraidHat == 1)
            BraidHat = 2 + randi(2);
        elseif(BraidHat==2)
            BraidHat = 4;
        elseif(BraidHat == 3)
            BraidHat = 1;
        elseif(BraidHat == 4)
        BraidHat = 3 - randi(2);
        end
        Braid(2,i) = BraidHat;
        BraidAgents(:,i) = BraidAgents(:,i)...
            -(BraidAgents(BraidHat,i)*((1:N).'==BraidHat)+BraidAgents(BraidHat+1,i)*((1:N).'==BraidHat+1))...
            +(BraidAgents(BraidHat+1,i)*((1:N).'==BraidHat)+BraidAgents(BraidHat,i)*((1:N).'==BraidHat+1));

    end
end

BraidPoint_X = squeeze(BraidPoints(1,:,1:end));
BraidPoint_Y = squeeze(BraidPoints(2,:,1:end));

for i = 1:N
    plot(BraidPoint_X(BraidAgents==i),...
        BraidPoint_Y(BraidAgents==i),'Color',c{i},'linewidth',2)
end

%% Setting up the simulation
break
% Simulation time
dt = 0.001; % 0.1 ms

% Terminal time
tf = 4*M; % seconds

% Time Vector 
t = 0:dt:tf;

% Allocate vector of linear velocities for N Agents, 2 per braid step.
V = zeros(N,2*M); 

% Agent Radius
r_agents = 0.07; % 7 cm

% Safety separation between agents
d_safe = 2*r_agents*1.1;% Twice their radius plus 10 percent satfety margin
j = 1;

% Allocating position and heading arrays for the Agents
X = zeros(N,length(t)); 
Y = zeros(N,length(t));
THETA = zeros(N,length(t));

% Initializing Agent Information
X(:,1) = squeeze(BraidPoints(1,:,1))';
Y(:,1) = squeeze(BraidPoints(2,:,1))';

% Distribute T evenly for now... to determine time alloted per braid step
tf_braid = tf/M; % seconds

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
HH = reshape(repmat(eye(3),1,2*(numTrap+1)),3,3,[]);
ii = 1;
ii_old = 0;
M_BraidCounter = 1; % <-----------------------------------------------------------------------------------------------------------------------
for i = 1:length(t)

    % At each path segment, compute the segment
    if(t(i)<=tf_braid*ii) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% change to braid counter <------------------------------------------------------
        % Check if we are starting at a new trapezoid, if so compute the
        % mapping from trapezoidal plane onto a rectagular plane
        if((ii_old ~= ii)&&((ii~=1)||(ii ~= numTrap+1)))
            % Select the corners of the rectangular plane. Let's select for
            % now the average of width and height rectangle.
            
            % First pick the correct trapezoid corners:[InnerBack, OuterBack, OuterFront, InnerFront]
            TrapezoidCorners = ...
                [BraidPoints(:,1,M_Rectangles+1+M_Trapezoids*(ii-1)),...
                 BraidPoints(:,end,M_Rectangles+1+M_Trapezoids*(ii-1)),...
                 BraidPoints(:,end,M_Rectangles+M_Trapezoids*ii),...
                 BraidPoints(:,1,M_Rectangles+M_Trapezoids*ii)];
            % Find the center point on each of the trapezoid parallel lines
            InnerCenterPoint = 0.5*(TrapezoidCorners(:,1)+TrapezoidCorners(:,4));
            OuterCenterPoint = 0.5*(TrapezoidCorners(:,2)+TrapezoidCorners(:,3));
            % Compute the lengths of the parallel lines
            InnerLineLength = norm(TrapezoidCorners(:,1)-TrapezoidCorners(:,4));
            OuterLineLength = norm(TrapezoidCorners(:,2)-TrapezoidCorners(:,3));
            % Compute the average length of the parallel lines
            RectangleLineLength = 0.5*(InnerLineLength+OuterLineLength);
            % Find the new corners from the center points: [InnerBack, OuterBack, OuterFront, InnerFront] 
            RectangleCorners = ...
                [InnerCenterPoint+0.5*RectangleLineLength*(TrapezoidCorners(:,1)-TrapezoidCorners(:,4)/InnerLineLength),...
                 OuterCenterPoint+0.5*RectangleLineLength*(TrapezoidCorners(:,2)-TrapezoidCorners(:,3)/OuterLineLength),...
                 OuterCenterPoint+0.5*RectangleLineLength*(TrapezoidCorners(:,3)-TrapezoidCorners(:,2)/OuterLineLength),...
                 InnerCenterPoint+0.5*RectangleLineLength*(TrapezoidCorners(:,4)-TrapezoidCorners(:,1)/InnerLineLength)];
                          
            A = [TrapezoidCorners(1,1) TrapezoidCorners(2,1) 1 0 0 0 -RectangleCorners(1,1)*TrapezoidCorners(1,1) -RectangleCorners(1,1)*TrapezoidCorners(2,1);
                 0 0 0 TrapezoidCorners(1,1) TrapezoidCorners(2,1) 1 -RectangleCorners(2,1)*TrapezoidCorners(1,1) -RectangleCorners(2,1)*TrapezoidCorners(2,1);
                 TrapezoidCorners(1,2) TrapezoidCorners(2,2) 1 0 0 0 -RectangleCorners(1,2)*TrapezoidCorners(1,2) -RectangleCorners(1,2)*TrapezoidCorners(2,2);
                 0 0 0 TrapezoidCorners(1,2) TrapezoidCorners(2,2) 1 -RectangleCorners(2,2)*TrapezoidCorners(1,2) -RectangleCorners(2,2)*TrapezoidCorners(2,2);
                 TrapezoidCorners(1,3) TrapezoidCorners(2,3) 1 0 0 0 -RectangleCorners(1,3)*TrapezoidCorners(1,3) -RectangleCorners(1,3)*TrapezoidCorners(2,3);
                 0 0 0 TrapezoidCorners(1,3) TrapezoidCorners(2,3) 1 -RectangleCorners(2,3)*TrapezoidCorners(1,3) -RectangleCorners(2,3)*TrapezoidCorners(2,3);
                 TrapezoidCorners(1,4) TrapezoidCorners(2,4) 1 0 0 0 -RectangleCorners(1,4)*TrapezoidCorners(1,4) -RectangleCorners(1,4)*TrapezoidCorners(2,4);
                 0 0 0 TrapezoidCorners(1,4) TrapezoidCorners(2,4) 1 -RectangleCorners(2,4)*TrapezoidCorners(1,4) -RectangleCorners(2,4)*TrapezoidCorners(2,4)];
            b = [RectangleCorners(:,1);RectangleCorners(:,2);RectangleCorners(:,3);RectangleCorners(:,4)]; % Same as RectangleCorners(:)...
            
            % The parameters of the transform in matrix form are given by
            H = reshape([(A\b);1],3,3)';
            % Let's store the transformation parameters
            HH(3,3,ii) = H;
        end
        
        % Filter the current braid points through the transform.
        % BraidCorners = [InnerBack,OuterBack,OuterFront,InnerFront]

        %%%%%%%%%%%%%%%%%%%%%%%%%%% Hypothesis %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Because the rectangular plane is symmetric around the center   %
        % vertical, and the braid points where distributed uniformly, the%
        % transformed waypoints will form an equidistant grid vertically %
        % and horizontally, i.e., the transformed waypoints will lie in  %
        % rectangles of equal height and width. We can therefore use only%
        % four sample points to determine the controller for all agents  %
        % and the transformation will change the controller back.        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%% Hypothesis %%%%%%%%%%%%%%%%%%%%%%%%%%%
        BraidCorners = [T(H,squeeze(BraidPoints(:,1,1+M_Trapezoids)))];
        
        
    else
        % We are starting a new path segment, update ii and reset i
        i = i - 1;
        ii_old = ii;
        ii = ii + 1;
    end
    
    
end







