alpha = 0:0.001:1;
% xhat = x0*(1-alpha)+y0*alpha;
xhat = [alpha.*1+(1-alpha).*0;
      0.5-0.5*cos(2*pi*6.*alpha)];
r_in = 1;
theta = 15/8*pi;
yhat = zeros(size(xhat));
for i=1:length(alpha)
   yhat(:,i) = [(r_in+xhat(2,i))*cos(theta*(1-xhat(1,i)));
                (r_in+xhat(2,i))*sin(theta*(1-xhat(1,i)))];
end

figure(2)
plot(xhat(1,:)-0.5,xhat(2,:)-0.5,'-b','linewidth',2)
hold on
plot(yhat(1,:),yhat(2,:),'-r','linewidth',2)
axis equal
