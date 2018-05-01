% zz
% basic understanding
% https://medium.com/@kastsiukavets.alena/kalman-filter-extended-kalman-filter-unscented-kalman-filter-dbbd929f83c5
% toy example
% https://towardsdatascience.com/wtf-is-sensor-fusion-part-2-the-good-old-kalman-filter-3642f321440

function [Target_estimate_potition_update, Covariance_update, difference]=KalmanFilter(Ture_measurement, Target_estimate_potition, Covariance,u_o)
x_o=Target_estimate_potition; % [px vx py vy]
z=Ture_measurement;
P=Covariance;
T = 5; % sample time
F = eye(4);
F(5) = T;
F(15) = T;
% F=[ 1 0 0 0; 0 1];
H=[1 -0; 0 1];
Q=0.1*eye(4); % noise
R=0.1*eye(4);


%Predict
x_o=F*x_o+u_o;             % 1
P=F*P*F'+Q;                % 2

%Update
temp = H*P*H'+R;           % 4
K=P*H*(temp)^(-1);         % 5 There is a difference between this equation and the one on the article
difference = z-H*x_o;      % 3
x_o=x_o + (K*difference);  % 6
P=(eye(2)-K*H)*P;          % 7

Target_estimate_potition_update=x_o;
Covariance_update = P;
end