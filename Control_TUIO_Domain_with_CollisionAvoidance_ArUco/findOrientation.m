function [localXdir,localYdir,localTheta] = findOrientation(x) % takes [2 x 4] matrix


center = sum(x,2)./4;
yvec = (x(:,1)+x(:,2))./2 - center;
xvec = (x(:,2)+x(:,3))./2 - center;
localYdir = yvec./norm(yvec);
localXdir = xvec./norm(xvec);
localTheta = [atan2(localXdir(2),localXdir(1));atan2(localYdir(2),localYdir(1))];


end