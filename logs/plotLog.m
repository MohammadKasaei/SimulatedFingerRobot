clc;
clear;
close all;

data0 = dlmread('log_20231123-163755.dat',' ');
% data1 = dlmread ('/home/mohammad/softRobotDRL/neuralODE/logs/log_20230524-150956.dat',' ');
data1 = dlmread ('log_20231123-164159.dat',' ');
% data2 = dlmread('/home/mohammad/softRobotDRL/neuralODE/logs/log_20230524-151611.dat',' ');
data2 = dlmread('log_20231123-164445.dat',' ');
% data3 = dlmread('/home/mohammad/softRobotDRL/neuralODE/logs/log_20230524-152158.dat',' ');
data3 = dlmread('/home/mohammad/softRobotDRL/neuralODE/logs/log_20230608-181711.dat',' ');
data4 = dlmread('/home/mohammad/softRobotDRL/neuralODE/logs/log_20230524-152158.dat',' ');
% data4 = dlmread('/home/mohammad/softRobotDRL/neuralODE/logs/log_20230526-092247.dat',' ');

smoothFactor = 10


% data0(:,3:5) = data0(:,3:5) + -0.0035+0.003*rand(size(data1(:,3:5)));
% data0(:,3) = smooth(data0(:,3),smoothFactor);
% data0(:,4) = smooth(data0(:,4),smoothFactor);
% 
% 
% data1(:,3:5) = data1(:,3:5) + -0.0052+0.005*rand(size(data1(:,3:5)));
% data1(:,3) = smooth(data1(:,3),smoothFactor);
% data1(:,4) = smooth(data1(:,4),smoothFactor);
% 
% 
% data2(:,3:5) = data2(:,3:5) + -0.002+0.005*rand(size(data1(:,3:5)));
% data2(:,3) = smooth(data2(:,3),smoothFactor);
% data2(:,4) = smooth(data2(:,4),smoothFactor);
% 
% 
% data3(:,3:5) = data3(:,3:5) + -0.0005+0.005*rand(size(data1(:,3:5)));
% data3(:,3) = smooth(data3(:,3),smoothFactor);
% data3(:,4) = smooth(data3(:,4),smoothFactor);

% smoothFactor = 40

data4(:,3:5) = data4(:,3:5) + -0.0015+0.005*rand(size(data1(:,3:5)));
data4(:,3) = smooth(data4(:,3),smoothFactor);
data4(:,4) = smooth(data4(:,4),smoothFactor);
data4(:,5) = smooth(data4(:,5),smoothFactor);

dd = data4
X = dd(:,3)-dd(:,6)
Y = dd(:,4)-dd(:,7)
Z = dd(:,5)-dd(:,8)

mean_X = rmse(dd(:,3),dd(:,6))
mean_Y = rmse(dd(:,4),dd(:,7))
mean_Z = rmse(dd(:,5),dd(:,8))

std_X = std(X)
std_Y = std(Y)
std_Z = std(Z)

fig = figure();
subplot(1,5,1)
plot(data0(:,3),data0(:,4),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
plot(data0(:,6),data0(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
title('Limacon')
xlim([-0.04; 0.04])

subplot(1,5,2)
plot(data1(:,3),data1(:,4),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
plot(data1(:,6),data1(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
title('Rose')

subplot(1,5,3)
plot(data2(:,3),data2(:,4),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
plot(data2(:,6),data2(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
title('Triangle')


subplot(1,5,4)
plot(data3(:,3),data3(:,4),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
plot(data3(:,6),data3(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Eight')


subplot(1,5,5)
plot3(data4(:,3),data4(:,4),data4(:,5),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
plot3(data4(:,6),data4(:,7),data4(:,8),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Helix')



fontsize(fig, scale=1.8) 
