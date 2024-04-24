clear all
TransformationFigure = figure('Name','Transformations','NumberTitle','off','color','w');%,'visible','off');
clf
set(gca,'LooseInset',get(gca,'TightInset'))
subplot(1,3,1)
hold on
% axis square
axis([0 3 -2 4 ])
set(gca,'PlotBoxAspectRatio',[ 1 1 1])
title('World')
set(gca,'DataAspectRatio',[ 1 1 1])

subplot(1,3,2)
hold on
% axis square_
axis([0 3 -2 4 ])
set(gca,'PlotBoxAspectRatio',[ 1 1 1])
title('Image')
set(gca,'DataAspectRatio',[ 1 1 1])


subplot(1,3,3)
hold on
axis([0 3 -2 4 ])
set(gca,'PlotBoxAspectRatio',[ 1 1 1])
title('World')
set(gca,'DataAspectRatio',[ 1 1 1])

% axis square

%% Define the quadrilaterals and the transformation
% First Quadrilateral's corners
w = [0;0];
x = [0;2];
y = [3;2];
z = [3;0];

% Second Quadrilateral's corners
% W = [0;-1];
% X = [0;2];
% Y = [2;4];
% Z = [2;-3];
W = [0;0];
X = [0;2];
Y = [3;4];
Z = [3;-2];

% Lets plot them
subplot(1,3,1)
plot([w(1) x(1) y(1) z(1) w(1)],[w(2) x(2) y(2) z(2) w(2)],'-b','linewidth',2)

subplot(1,3,2)
plot([W(1) X(1) Y(1) Z(1) W(1)],[W(2) X(2) Y(2) Z(2) W(2)],'-r','linewidth',2)

subplot(1,3,3)
plot([w(1) x(1) y(1) z(1) w(1)],[w(2) x(2) y(2) z(2) w(2)],'-g','linewidth',2)


% Form the 8x8 system of equations to solve and the inverse formulation

A = [w(1) w(2) 1 0 0 0 -W(1)*w(1) -W(1)*w(2);
     0 0 0 w(1) w(2) 1 -W(2)*w(1) -W(2)*w(2);
     x(1) x(2) 1 0 0 0 -X(1)*x(1) -X(1)*x(2);
     0 0 0 x(1) x(2) 1 -X(2)*x(1) -X(2)*x(2);
     y(1) y(2) 1 0 0 0 -Y(1)*y(1) -Y(1)*y(2);
     0 0 0 y(1) y(2) 1 -Y(2)*y(1) -Y(2)*y(2);
     z(1) z(2) 1 0 0 0 -Z(1)*z(1) -Z(1)*z(2);
     0 0 0 z(1) z(2) 1 -Z(2)*z(1) -Z(2)*z(2)];
b = [W;X;Y;Z];


Ainv = [W(1) W(2) 1 0 0 0 -w(1)*W(1) -w(1)*W(2);
        0 0 0 W(1) W(2) 1 -w(2)*W(1) -w(2)*W(2);
        X(1) X(2) 1 0 0 0 -x(1)*X(1) -x(1)*X(2);
        0 0 0 X(1) X(2) 1 -x(2)*X(1) -x(2)*X(2);
        Y(1) Y(2) 1 0 0 0 -y(1)*Y(1) -y(1)*Y(2);
        0 0 0 Y(1) Y(2) 1 -y(2)*Y(1) -y(2)*Y(2);
        Z(1) Z(2) 1 0 0 0 -z(1)*Z(1) -z(1)*Z(2);
        0 0 0 Z(1) Z(2) 1 -z(2)*Z(1) -z(2)*Z(2)];
binv = [w;x;y;z];

% The solutions in matrix form
H = reshape([(A\b);1],3,3)';
Hinv = reshape([(Ainv\binv);1],3,3)';

% The transformation at a point x is given through H by
T = @(H,x) [eye(2) zeros(2,1)]*(H*[x;1]/(H(3,:)*[x;1]));

%% Let's test out the transformation
% First draw a straight line in the blue rectangle and check how it looks
% on the red trapezoid
%
% Let's parametrize all curves as a function of time
t = 0:0.01:1;
% A line starting at w and ending at y.
Line1_World = w*(1-t)+y*t;%[t.*y(1)+(1-t).*w(1);t.*y(2)+(1-t).*w(2)];

subplot(1,3,1)
plot(Line1_World(1,:),Line1_World(2,:),':b')
T_Line1_World = zeros(2,length(t));
Tinv_T_Line1_World = zeros(2,length(t));
for i = 1:length(t);
    T_Line1_World(:,i) = T(H,Line1_World(:,i));
    Tinv_T_Line1_World(:,i)= T(Hinv,T_Line1_World(:,i));
end
subplot(1,3,2)
plot(T_Line1_World(1,:),T_Line1_World(2,:),':r')

subplot(1,3,3)
plot(Tinv_T_Line1_World(1,:),Tinv_T_Line1_World(2,:),':g')


% Now, let's draw a straight line on the red trapezoid and plot it back on
% the blue rectangle through the inverse transformation
Circle_Radius = 0.5;
Circle_Center = T(H,0.5*(y-w));
Circle_Image = Circle_Radius.*[cos(2*pi*t);sin(2*pi*t)]+Circle_Center*ones(size(t));
Line2_Image = Z*(1-t)+X*t;%[t.*X(1)+(1-t).*Z(1);t.*X(2)+(1-t).*Z(2)];
subplot(1,3,2)
plot(Line2_Image(1,:),Line2_Image(2,:),'--r')
plot(Circle_Image(1,:),Circle_Image(2,:),'-r')
Tinv_Line2_Image = zeros(2,length(t));
Tinv_Circle_Image = zeros(2,length(t));
for i = 1:length(t);
    Tinv_Line2_Image(:,i) = T(Hinv,Line2_Image(:,i));
    Tinv_Circle_Image(:,i)= T(Hinv,Circle_Image(:,i));
end
subplot(1,3,3)
plot(Tinv_Line2_Image(1,:),Tinv_Line2_Image(2,:),'--g')
plot(Tinv_Circle_Image(1,:),Tinv_Circle_Image(2,:),'-g')

% Lets try a curve, say a sinusoid
Periods = 6;
Line3_World = [t.*w(1)+(1-t).*z(1);
              (x(2)-w(2))/2+(x(2)-w(2))/2*cos(2*pi*Periods.*t)];
subplot(1,3,1)
plot(Line3_World(1,:),Line3_World(2,:),'-b')
T_Line3_World = zeros(2,length(t));
Tinv_T_Line3_World = zeros(2,length(t));
for i = 1:length(t);
    T_Line3_World(:,i) = T(H,Line3_World(:,i));
    Tinv_T_Line3_World(:,i) = T(Hinv,T_Line3_World(:,i));
end
set(gca,'LooseInset',get(gca,'TightInset'))
axis off

subplot(1,3,2)
plot(T_Line3_World(1,:),T_Line3_World(2,:),'-r')
set(gca,'LooseInset',get(gca,'TightInset'))
axis off

subplot(1,3,3)
plot(Tinv_T_Line3_World(1,:),Tinv_T_Line3_World(2,:),'-g')
set(gca,'LooseInset',get(gca,'TightInset'))
axis off

annotation('arrow', [.36 .39], [0.5 0.5],...
    'HeadWidth',15,'LineWidth',2.5);
annotation('textbox', [0.36 0.525 0.03 0.1],...
    'FontSize',18,'interpreter','latex',...
    'HorizontalAlignment','center','VerticalAlignment','bottom',...
    'String','$H$','LineStyle','none');

annotation('arrow', [0.64 .67], [0.5 0.5],...
    'HeadWidth',15,'LineWidth',2.5);
annotation('textbox', [0.64 0.525 0.03 0.1],...
    'FontSize',18,'interpreter','latex',...
    'HorizontalAlignment','center','VerticalAlignment','bottom',...
    'String','$H^{-1}$','LineStyle','none');
