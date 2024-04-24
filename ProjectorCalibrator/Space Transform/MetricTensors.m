%% Metric Tensors
% This scripts is meant to compute the lengths of paths produces in
% TestingQuads.m via the use of the metric tensor derived.
clear all
close all
clc
TestingQuads

T = @(H,x) [eye(2) zeros(2,1)]*(H*[x;1]/(H(3,:)*[x;1]));

% Define the partial derivatives of the transformation
dTdx1 = @(H,x) ...
    [(H(1,1)-H(1,3)*H(3,1)+(H(1,1)*H(3,2)-H(1,2)*H(3,1))*x(2))/(H(3,1:2)*x+1)^2;
     (H(2,1)-H(2,3)*H(3,1)+(H(2,1)*H(3,2)-H(2,2)*H(3,1))*x(2))/(H(3,1:2)*x+1)^2]; 

dTdx2 = @(H,x) ...
    [(H(1,2)-H(1,3)*H(3,2)+(H(1,2)*H(3,1)-H(1,1)*H(3,2))*x(1))/(H(3,1:2)*x+1)^2;
     (H(2,2)-H(2,3)*H(3,2)+(H(2,2)*H(3,1)-H(2,1)*H(3,2))*x(1))/(H(3,1:2)*x+1)^2]; 

% Define the Metric Tensor (Tensor)
M = @(H,x) [dTdx1(H,x).'*dTdx1(H,x),dTdx1(H,x).'*dTdx2(H,x);
            dTdx1(H,x).'*dTdx2(H,x),dTdx2(H,x).'*dTdx2(H,x)];
% M = @(H,x,i) ...
% [((H(1,1)-H(1,3)*H(3,1)+(H(1,1)*H(3,2)-H(1,2)*H(3,1))*x(2,i))/(H(3,1:2)*x(:,i)+1)^2)^2+...
%  ((H(2,1)-H(2,3)*H(3,1)+(H(2,1)*H(3,2)-H(2,2)*H(3,1))*x(2,i))/(H(3,1:2)*x(:,i)+1)^2)^2,...
%  ...
%  ((H(1,1)-H(1,3)*H(3,1)+(H(1,1)*H(3,2)-H(1,2)*H(3,1))*x(2,i))/(H(3,1:2)*x(:,i)+1)^2)*...
%  ((H(1,2)-H(1,3)*H(3,2)-(H(1,1)*H(3,2)-H(1,2)*H(3,1))*x(1,i))/(H(3,1:2)*x(:,i)+1)^2)+...
%  ((H(2,1)-H(2,3)*H(3,1)+(H(2,1)*H(3,2)-H(2,2)*H(3,1))*x(2,i))/(H(3,1:2)*x(:,i)+1)^2)*...
%  ((H(2,2)-H(2,3)*H(3,2)-(H(2,1)*H(3,2)-H(2,2)*H(3,1))*x(1,i))/(H(3,1:2)*x(:,i)+1)^2);...
%  ...
%  ((H(1,1)-H(1,3)*H(3,1)+(H(1,1)*H(3,2)-H(1,2)*H(3,1))*x(2,i))/(H(3,1:2)*x(:,i)+1)^2)*...
%  ((H(1,2)-H(1,3)*H(3,2)-(H(1,1)*H(3,2)-H(1,2)*H(3,1))*x(1,i))/(H(3,1:2)*x(:,i)+1)^2)+...
%  ((H(2,1)-H(2,3)*H(3,1)+(H(2,1)*H(3,2)-H(2,2)*H(3,1))*x(2,i))/(H(3,1:2)*x(:,i)+1)^2)*...
%  ((H(2,2)-H(2,3)*H(3,2)-(H(2,1)*H(3,2)-H(2,2)*H(3,1))*x(1,i))/(H(3,1:2)*x(:,i)+1)^2),...
%  ...
%  ((H(1,2)-H(1,3)*H(3,2)-(H(1,1)*H(3,2)-H(1,2)*H(3,1))*x(1,i))/(H(3,1:2)*x(:,i)+1)^2)^2+...
%  ((H(2,2)-H(2,3)*H(3,2)-(H(2,1)*H(3,2)-H(2,2)*H(3,1))*x(1,i))/(H(3,1:2)*x(:,i)+1)^2)^2 ];

% Compute the rate of change of the lines in the world plane

dotL1 = -[(y(1)-w(1)).*ones(size(t));(y(2)-w(2)).*ones(size(t))];
subplot(1,3,1)
% text(y(1),y(2)+0.1,'Start here for 1.');
% text(w(1), w(2)-0.1,'End here for 1.');
% T_Line1_World = f;
sL1 = 0;
SL1 = 0;
M_Stored = zeros(2,2,length(t)-1);
for i = 1:length(t)-1
    sL1 = sL1 + (dotL1(:,i).'*dotL1(:,i))^0.5*(t(i+1)-t(i));
    SL1 = SL1 + (dotL1(:,i).'*M(H,Line1_World(:,i))*dotL1(:,i))^0.5*(t(i+1)-t(i));
    M_Stored(:,:,i) = M(H,Line1_World(:,i));
end
sL2 = norm(Line1_World(:,end)-Line1_World(:,1));
SL2 = norm(T_Line1_World(:,end)-T_Line1_World(:,1));
err = abs(sL1-sL2);
Err = abs(SL1-SL2);

% Let's do the sinusoidal curves arc length for fun
dotL3 = [(w(1)-z(1)).*ones(size(t));-2*pi*Periods*(x(2)-w(2))/2.*sin(2*pi*Periods.*t)];
sC1 = 0;
SC1 = 0;
SC2 = 0;
for i = 1:length(t)-1
    sC1 = sC1 + (dotL3(:,i).'*dotL3(:,i))^0.5*(t(i+1)-t(i)); % Arc length formular for sinusoid in world plane
    SC1 = SC1 + (dotL3(:,i).'*M(H,Line3_World(:,i))*dotL3(:,i))^0.5*(t(i+1)-t(i)); % Metric Tensor length in Image Plane
    SC2 = SC2 + norm(T_Line3_World(:,i+1)-T_Line3_World(:,i)); % Summing up length differences directly
end
Err2 = abs(SC1-SC2);

% Experimenting the other way... from trapezoid to square
dot_T_Line1_World = zeros(size(dotL1));%(T_Line1_World(:,end)-T_Line1_World(:,1))*ones(size(t));
ArcLength_Tinv_T_Line1 = 0;
% tL1_EulerApprox = T_Line1_World;
% ttL1_EulerApprox = (T(Hinv,T_Line1_World(:,1)))*ones(1,size(T_Line1_World,2));
% tL1_EulerApprox_ImagePlane = M(Hinv,tL1,1)*dot_tL1(:,1);
Minv_Stored = zeros(2,2,length(t)-1);

subplot(1,3,2)
CarWorld = plot(T_Line1_World(1,1),T_Line1_World(2,1),'ko');
subplot(1,3,3)
CarImage = plot(Tinv_T_Line1_World(1,1),Tinv_T_Line1_World(2,1),'ko');
% pause
for i = 1:length(t)-1
    
    %     ttL1_EulerApprox(:,i+1) = ttL1_EulerApprox(:,i) +([dTdx1(Hinv,tL1_EulerApprox(:,i)),dTdx2(Hinv,tL1_EulerApprox(:,i))]*dot_tL1(:,i))*(t(i+1)-t(i));
    %     tL1_EulerApprox(:,i+1) = tL1_EulerApprox(:,i) +dot_tL1(:,i)*(t(i+1)-t(i));
    dot_T_Line1_World(:,i) = [dTdx1(H,Line1_World(:,i)),dTdx2(H,Line1_World(:,i))]*dotL1(:,i);
    %     tL1_EulerApprox_ImagePlane = tL1_EulerApprox_ImagePlane +dot_tL1(:,i)*(a(i+1)-a(i));
    ArcLength_Tinv_T_Line1 = ArcLength_Tinv_T_Line1 + ((dot_T_Line1_World(:,i).'*M(Hinv,T_Line1_World(:,i))*dot_T_Line1_World(:,i)))^0.5*(t(i+1)-t(i));
    Minv_Stored(:,:,i) = (M(Hinv,T_Line1_World(:,i)));
    set(CarWorld,'xData',T_Line1_World(1,i),'yData',T_Line1_World(2,i))
    set(CarImage,'xData',Tinv_T_Line1_World(1,i),'yData',Tinv_T_Line1_World(2,i))
%     figure(transformationfigure)
        pause(t(i+1)-t(i));
        drawnow
    %     stL1 = stL1 + (dot_tL1(:,i).'/(M(H,T(H,L1(:,i))))*dot_tL1(:,i))^0.5*(a(i+1)-a(i));
end
%     subplot(1,3,2)
%     plot(tL1_EulerApprox(1,:),tL1_EulerApprox(2,:),'ko')
%     subplot(1,3,3)
%     plot(ttL1_EulerApprox(1,:),ttL1_EulerApprox(2,:),'ko')
    figure(TransformationFigure)