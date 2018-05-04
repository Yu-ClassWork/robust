% kalman filter implementation
% first start off with 1D implementation

% what is the true trajectory
% expected trajectory (inputs): 5' right -> 2' left -> 7' right
close all; clear all; clc;
% load('2018-02-25 18-10-55.tlog.mat');
% load('2018-02-25 18-23-56.tlog.mat');
% load('2018-02-25 18-29-57.tlog.mat');
load('2018-02-25 18-35-44.tlog.mat');

% Initial covariance
covariance = 1e-3*eye(2);
tempNoise = normrnd(0, 1);
u_o = [tempNoise; tempNoise];
P = covariance;
Q=0.1*eye(2);
F = eye(2);
% F(5) = 1;
% F(15) = 1;
H=eye(2);
R=0.1*eye(2);
x_o = [x_mavlink_local_position_ned_t(1,2) y_mavlink_local_position_ned_t(1,2)];
x = x_o';
xUpdate = x;
zMeas = [];

% [xGround, yGround] = deg2utm(lat_mavlink_gps_raw_int_t(:,2)/1000000, lon_mavlink_gps_raw_int_t(:,1)/1000000)

% plotXY = [];
for i = 1:length(x_mavlink_local_position_ned_t)-1
    vel = [vx_mavlink_local_position_ned_t(i,2) vy_mavlink_local_position_ned_t(i,2)]';
    deltTime = [vx_mavlink_local_position_ned_t(i+1,1)-vx_mavlink_local_position_ned_t(i,1) vy_mavlink_local_position_ned_t(i+1,1)-vy_mavlink_local_position_ned_t(i,1)]';
    tempNoise1 = normrnd(0, 1);
    tempNoise2 = normrnd(0, 1);
    u_o = [tempNoise1; tempNoise2];
    tempX=F*x(:,end)+ vel.*deltTime +u_o; % 1
    P=F*P*F'+Q; % 2
    x = [x, tempX];
    
    z = [x_mavlink_local_position_ned_t(i+1,2) y_mavlink_local_position_ned_t(i+1,2)]';
    
    temp = H*P*H'+R; % 4
    K=P*H*(temp)^(-1); % 5 There is a difference between this equation and the one on the article
    difference = z-H*tempX; % 3
    tempX1=tempX + (K*difference); % 6
    P=(eye(2)-K*H)*P; % 7
    zMeas = [zMeas,z];
    xUpdate = [xUpdate, tempX1];
    
end
% plotXY(1,:)

figure()
% hold on
% plot(xGround, yGround, '--xb')
plot(lon_mavlink_gps_raw_int_t(:,2), lat_mavlink_gps_raw_int_t(:,2))
figure()

plot(xUpdate(2,:),xUpdate(1,:), '--or')
% plot(zMeas(2,:),zMeas(1,:), '--*g')
% plot( y_mavlink_local_position_ned_t(:,2), x_mavlink_local_position_ned_t(:,2),'--og')
axis equal