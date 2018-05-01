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
covariance = diag([1e-3 1e-3 1e-3 1e-3]);
u_o = [1e-3; 1e-3; 1e-3; 1e-3];
P = covariance;
Q=0.1*eye(4);
F = eye(4);
F(5) = 1;
F(15) = 1;
H=eye(4);
R=0.1*eye(4);
x_o = [x_mavlink_local_position_ned_t(1,2) vx_mavlink_local_position_ned_t(1,2) y_mavlink_local_position_ned_t(1,2) vy_mavlink_local_position_ned_t(1,2)];
x = x_o';
xUpdate = x;

% plotXY = [];
for i = 1:length(x_mavlink_local_position_ned_t)
    %     [target_update, covariance_update, difference] = KalmanFilter(ture_measurement, target_estimate_potition, covariance,u_o);
    x(:,end)
    
    tempX=F*x(:,end)+u_o;             % 1
    P=F*P*F'+Q;                % 2
    x = [x, tempX];
    
    z = [x_mavlink_local_position_ned_t(i,2) vx_mavlink_local_position_ned_t(i,2) y_mavlink_local_position_ned_t(i,2) vy_mavlink_local_position_ned_t(i,2)]';
    
    temp = H*P*H'+R;           % 4
    K=P*H*(temp)^(-1);         % 5 There is a difference between this equation and the one on the article
    difference = z-H*tempX;      % 3
    tempX1=tempX + (K*difference);  % 6
    P=(eye(4)-K*H)*P;          % 7
    xUpdate = [xUpdate, tempX1];
    
end
% plotXY(1,:)

figure()
hold on
% plot(x(1,:), x(3,:), '--xb')
plot(xUpdate(3,:),xUpdate(1,:), '--or')
plot( y_mavlink_local_position_ned_t(:,2), x_mavlink_local_position_ned_t(:,2),'--og')
axis equal