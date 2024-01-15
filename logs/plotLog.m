clc;
clear;
close all;

data0 = dlmread('log_20231123-163755.dat',' ');
data0_o = dlmread('log_20231124-143525_Limacon.dat',' ');
% data1 = dlmread ('log_20231123-164159.dat',' ');
% data1_o = dlmread ('log_20231124-143800_Rose.dat',' ');
data1 = dlmread('log_20240115-094510_Square.dat',' ');
data1_o = dlmread ('log_20240115-094222_Square.dat',' ');

data2 = dlmread('log_20231123-164445.dat',' ');
data2_o = dlmread('log_20231124-144220_Eight_Figure.dat',' ');
data3 = dlmread('log_20231124-134739_Circle.dat',' ');
% data3_o = dlmread('log_20231124-144504_Circle.dat',' ');
% data3_o = dlmread('log_20231127-133756_Circle.dat',' ');
data3_o = dlmread('log_20231127-135141_Circle.dat',' ');
data4 = dlmread('log_20231124-135653_Moveing_Square.dat',' ');
data4_o = dlmread('log_20231124-143208_Moveing_Square.dat',' ');



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
% 
% data4(:,3:5) = data4(:,3:5) + -0.0015+0.005*rand(size(data1(:,3:5)));
% data4(:,3) = smooth(data4(:,3),smoothFactor);
% data4(:,4) = smooth(data4(:,4),smoothFactor);
% data4(:,5) = smooth(data4(:,5),smoothFactor);
% 
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
box on
plot(data0_o(:,3),data0(:,4),'b-',LineWidth=3,DisplayName='x');
plot(data0(:,6),data0(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
title('Limacon')
xlim([-0.06; 0.04])

subplot(1,5,2)
plot(data1(:,3),data1(:,4),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
box on
plot(data1_o(:,3),data1_o(:,4),'b-',LineWidth=3,DisplayName='x');
plot(data1(:,6),data1(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
title('Square')

subplot(1,5,3)
plot(data2(:,3),data2(:,4),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
box on
plot(data2_o(:,3),data2_o(:,4),'b-',LineWidth=3,DisplayName='x');
plot(data2(:,6),data2(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
title('Eight')


subplot(1,5,4)
plot(data3(:,3),data3(:,4),'r-',LineWidth=3,DisplayName='x');
hold on
grid on
box on
plot(data3_o(:,3),data3_o(:,4),'b-',LineWidth=3,DisplayName='x');
plot(data3(:,6),data3(:,7),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Circle')


subplot(1,5,5)
plot3(data4(:,3),data4(:,4),data4(:,5),'r-',LineWidth=3,DisplayName='x');
hold on
plot3(data4_o(:,3),data4_o(:,4),data4_o(:,5),'b-',LineWidth=3,DisplayName='x');
grid on
plot3(data4(:,6),data4(:,7),data4(:,8),'k--',LineWidth=2,DisplayName='xd');
% legend
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Square Helix')



fontsize(fig, scale=1.8) 
