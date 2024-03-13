clear all, close all, clc


%% Load Data

% load('sc_data_noesc2.mat') % Faster with sin
 load('sc_data_noesc3.mat') % slow

%x = w_data(1:3, 10000:end)';
x = w_data';
dt = 0.01; % sample time for data
n=3; 
%xTrain = x(1:100, :);


%% compute derivative
for i=1:3
    dx(:,i) = diff(x(:,i))/dt;%diff(x(1:1000,i))/dt; % only using part of the data for training
end

dx = [dx; dx(end,:)];

%% part d
% lets add noise for fun :)
% noise for SYS1
% noise = normrnd(0.1,0.2, size(x,1), size(x,2));

% noise for SYS2
%noise = normrnd(0.1,0.2, size(x,1), size(x,2))/100;

% x = x+noise;
% 
% 
% for i=1:3
%     dx(:,i) = diff(x(:,i))/dt;
% end
% dx = [dx; dx(end,:)];


%% Build library and compute sparse regression
polyorder = 1; % up to third order polynomials
usesine = 1;
Theta = poolData(x, n, polyorder, usesine);
lambda = 0.01;      % lambda is our sparsification knob.
Xi = sparsifyDynamics(Theta,dx,lambda,n)
poolDataLIST({'x','y','z'},Xi,n,polyorder,usesine);

t_f = 600;
t = 0:dt:t_f;
tspan = t; %t(1:2000);
options = odeset('RelTol',1e-6,'AbsTol',1e-6*ones(1,n));
x0 = x(1,:)';% initial conditions
[tD,xD]=ode45(@(t,x)sparseGalerkin(t,x,Xi,polyorder,usesine),tspan,x0,options);
% 
% 
figure
plot(t(1:length(tspan)),x(1:length(tspan), :)*180/pi, 'LineWidth', 5)
hold on
plot(tD, xD*180/pi, '--', 'LineWidth', 2)
legend('x true', 'y true', 'z true', 'x model', 'y model', 'z model')
xlabel('Time, s','FontSize',14)
ylabel('Angular Rates, deg/s','FontSize',14)
if usesine == 0
        usedsine = "no"
    else
        usedsine = "yes"
end
titlestring = sprintf("Spacecraft Rotation Rates Over Time\nPolynomial Order = %d, Sparsification" + ...
    " = %0.2f, Sine Used = "+usedsine,polyorder,lambda)
title(titlestring,'FontSize',16)

