clear all, close all,  clc
% for each part, uncomment and run entire code

%% Load Data

%load('SYS1.mat')
%load('SYS2.mat')
% load('bathymetry.mat')
load('bathymetry_no_noise.mat')
dt = 0.01; % sample time for data
n=3; 

%% Compute Derivative


%% part a 

% Beta = [10; 28; 8/3]; % Lorenz's parameters (chaotic)
% for i=1:length(x)
%     dx(i,:) = lorenz(0,x(i,:),Beta);
% end


%////////////////////////////////////
%% part b

% for i=1:length(x)
%     dx(i,:) = dynamic_p5(0,x(i,:));
% end

%% part c

% for i=1:3
%     dx(:,i) = diff(x(:,i))/dt;
% end
% dx = [dx; dx(end,:)];


%% part d
% lets add noise for fun :)
% noise for SYS1
noise = normrnd(0.1,0.2, size(x,1), size(x,2));

%noise for SYS2
%noise = normrnd(0.1,0.2, size(x,1), size(x,2))/100;

%x = x+noise;

% for i=1:3
%     dx(:,i) = diff(x(:,i))/dt;
% end
% dx = [dx; dx(end,:)];

dx = x;
% storage for filtered data
dx_filter = zeros(size(dx,1), size(dx,2));
xyz_str = ['x','y','z'];

% plot data
for i = 1:size(dx,2)
    % plot Original data
    figure 
    plot(dx(:,i)); hold on
    
    % filter data using Savitzky Golay
    rd = 2; % polynomial order (use 2 for SYS2.mat)
    fl = 21; % length of data frames (USE 21 FOR SYS2.mat)   
    dx_filter(:,i) = sgolayfilt(dx(:,i),rd,fl);
    
    % plot filtered data
    plot(dx_filter(:,i)); hold off
    title_str = sprintf('Filter d%1$s', xyz_str(i));
    title(title_str)
    grid
    %xlim([1000 2000]) % Add for SYS1.MAT TO BETTER SEE NOISE
    legend({'Original', 'Filtered'},'Location','southeast')
end

%% Build library and compute sparse regression
polyorder = 2; % up to third order polynomials
usesine = 1;
Theta = poolData(x,n,polyorder,usesine); 
lambda = 0.01;      % lambda is our sparsification knob.
Xi = sparsifyDynamics(Theta,dx_filter,lambda,n)
poolDataLIST({'x','y','z'},Xi,n,polyorder,usesine);

