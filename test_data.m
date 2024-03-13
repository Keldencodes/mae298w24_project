%Get some real clean dynamics
%AZ, Feb 15 2024

%sensors
IMU_noise = 0 * (pi/180); %error, deg/s -> rad/s, 1 sigma
att_noise = 1 * (pi/180); %error, deg -> rad
IMU_bias = [0; 0; 0] * (pi/180); %bias -> rad

%initial attitude and angular rates
q = [-0.145392350642603;0;0;0.989374077068233];  %arbitrary initial attitude, scalar last
w = 1*[1;1;1]*pi/180;  %deg/s -> rad/sec

%target rotation
ang = 135 * (pi/180); % deg -> rad
R1 = [1 0 0; 0 cos(ang) -1*sin(ang); 0 sin(ang) cos(ang)];
R2 = [cos(ang) 0 sin(ang); 0 1 0; -1*sin(ang) 0 cos(ang)];
R3 = [cos(ang) -1*sin(ang) 0; sin(ang) cos(ang) 0; 0 0 1];
q_i = DCM_to_quat_v2(R3);
q_t = [q_i(4)   q_i(3)  -q_i(2)  -q_i(1);    %q_target
      -q_i(3)   q_i(4)   q_i(1)  -q_i(2);
       q_i(2)  -q_i(1)   q_i(4)  -q_i(3);
       q_i(1)   q_i(2)   q_i(3)   q_i(4)]*q;

%controller gain
rw_kd_gain = 0.6*[0.002149;  0.002058;  0.002291];
rw_kp_gain = 5.0*[0.00000841; 0.00000805; 0.00000896];

ctl_option_rw = 1; %0 = no RW, 1 = ramp up, 2 = controller

rw_w = 0;
rw_w_prev = 0;

% q_0 = DCM_to_quat_v2(eye(3));
% q_180x = DCM_to_quat_v2([1 0 0; 0 -1 0; 0 0 -1]);
% q_180y = DCM_to_quat_v2([-1 0 0; 0 1 0; 0 0 -1]);
% q_t = [q_180x(4)   q_180x(3)  -q_180x(2)  -q_180x(1);  
%       -q_180x(3)   q_180x(4)   q_180x(1)  -q_180x(2);
%        q_180x(2)  -q_180x(1)   q_180x(4)  -q_180x(3);
%        q_180x(1)   q_180x(2)   q_180x(3)   q_180x(4)]*q;
% q_t = q;
% time
t_f = 600; %length of sim, seconds
dt = 0.01; %time step. May misconverge at larger time steps or at faster angular rates
t = 0:dt:t_f;
n = length(t);

% constants
% mass properties for a 3U CubeSat
% MOI = [0.040737932450, 0.000745007330, -6.2158270e-05;
%        0.000745007330, 0.007688814240,  4.0867890e-05;
%       -6.2158270e-05,  4.0867890e-05,   0.040540656630];
% mass = 3.6512;                             %kg
% cg_offset = [0.00080, -0.00320, 0.00110];  %m

% mass properties for a 1U CubeSat
MOI = 10^-9 * [1869276   35027   -9213;   %estimated MOI for a particular 1U spacecraft
                 35027 1790125   -8048;
                 -9213   -8048 1992745];
mass = 0.975;
cg_offset = 10^-3 * [-0.5 1.8 -0.6];

% update these values with actual values of your reaction wheels
rw_J            = 10^-5; %kg*m2
rw_max_tor      = 2 * 10^-3; %Nm
rw_rot_axis     = eye(3);
rw_max_omega    = 5000 * (pi/30); % RPM -> rad/s
rw_min_omega    =  100 * (pi/30); % RPM -> rad/s
rw_tau          = 0.5;  %first order time constant
rw_cmd          = [0;0;0];  % -1 to 1 for wheel command
rw_freq         = 5; % RW controller update rate, Hz
rw_dt           = 1/rw_freq;
time_last_rw_cmd = 0;

rw_ramp_start = 20;     %time to start  linearly ramping wheel command
rw_ramp_end   = 50;     %time to finish linearly ramping wheel command
rw_ramp_peak  = 50/100; %percent of maximum wheel speed to ramp up to

%rw_kd_gain = 0.0020;
%rw_kp_gain = 0.0006;

tor_cmd = 0; % desired torque, Nm
cmd_resolution = 9; %number of bits

data_out = zeros(18, n);
w_dot_AB = zeros(3,3);
q_dot_AB = zeros(4,3);
AB = [23/12; -16/12; 5/12];
for i=1:n
%     e1 = q(1); e2 = q(2); e3 = q(3); eta = q(4);
%     R_eci2body = [e1^2-e2^2-e3^2+eta^2, 2*(e1*e2+eta*e3),        2*(e1*e3-eta*e2);
%                   2*(e1*e2-eta*e3), -1*e1^2+e2^2-e3^2+eta^2, 2*(e2*e3+eta*e1);
%                   2*(e1*e3+eta*e2),  2*(e2*e3-eta*e1), -1*e1^2-e2^2+e3^2+eta^2];
    
    
    %calculate wheel command
    if t(i) > 10
        ctl_option_rw = 1;
    end
%     if t(i) > 40
%         ctl_option_rw = 0;
%     end
    
    if t(i) - time_last_rw_cmd > rw_dt
        switch ctl_option_rw
            case 0
                %option 0: no wheels
                rw_cmd = [0;0;0];
            case 1
                %option 1: wheel ramp up
                if t(i) > rw_ramp_start
                    rw_cmd(2) = rw_ramp_peak * (t(i) - rw_ramp_start)/(rw_ramp_end-rw_ramp_start);
                end
                if t(i) > rw_ramp_end
                    rw_cmd(2) = rw_ramp_peak;
                end
            case 2
                %option 2: control
                ang = att_noise * randn;
                R1 = [1 0 0; 0 cos(ang) -1*sin(ang); 0 sin(ang) cos(ang)];
                ang = att_noise * randn;
                R2 = [cos(ang) 0 sin(ang); 0 1 0; -1*sin(ang) 0 cos(ang)];
                ang = att_noise * randn;
                R3 = [cos(ang) -1*sin(ang) 0; sin(ang) cos(ang) 0; 0 0 1];
                q_e = DCM_to_quat_v2(R1*R2*R3);
                q_meas = [q_e(4)   q_e(3)  -q_e(2)  -q_e(1);  %q_e
                         -q_e(3)   q_e(4)   q_e(1)  -q_e(2);
                          q_e(2)  -q_e(1)   q_e(4)  -q_e(3);
                          q_e(1)   q_e(2)   q_e(3)   q_e(4)]*q;
                q_er = [q_t(4)   q_t(3)  -q_t(2)  -q_t(1);  %q_error
                       -q_t(3)   q_t(4)   q_t(1)  -q_t(2);
                        q_t(2)  -q_t(1)   q_t(4)  -q_t(3);
                        q_t(1)   q_t(2)   q_t(3)   q_t(4)]*q_meas; 
                sq = [q_er(1:3); 1+q_er(4)]/sqrt(2*(1+q_er(4)));
                MRP = sq(1:3)/sq(4); %proportional error
                if norm(MRP) > 1
                    MRP = -MRP/(MRP'*MRP);
                end
                w_meas = w + IMU_noise * randn(3,1) + IMU_bias;
                tor_cmd = rw_kp_gain .* MRP + rw_kd_gain .* w_meas;
                for j=1:3
                    tor_cmd(j) = min(   rw_max_tor, tor_cmd(j));
                    tor_cmd(j) = max(-1*rw_max_tor, tor_cmd(j));
                    rw_cmd(j) = rw_cmd(j) + (dt/rw_J)*tor_cmd(j);
                    rw_cmd(j) = floor(rw_cmd(j) * 2^cmd_resolution) / (2^cmd_resolution);  %resolution error
                    rw_cmd(j) = max(rw_cmd(j), -1); %command within bounds
                    rw_cmd(j) = min(rw_cmd(j),  1); %command within bounds 
                end     
        end
        time_last_rw_cmd = t(i);
    end
    
    % wheel dynamics
    % note, I'm not applying the dead zone or enforcing maximum torque
    rw_w = rw_tau * rw_max_omega * rw_cmd + (1-rw_tau) * rw_w;
    rw_torque = rw_rot_axis * -1*rw_J*(rw_w - rw_w_prev);
    rw_w_prev = rw_w;

    %spacecraft dynamics
    wx = w(1); wy = w(2); wz = w(3);
    OMEGA = [   0,    wz, -1*wy, wx;
            -1*wz,     0,    wx, wy;
               wy, -1*wx,     0, wz;
            -1*wx, -1*wy, -1*wz,  0];
    q_dot = 0.5*OMEGA*q;
    q_dot_AB(:, 2:3) = q_dot_AB(:, 1:2);
    q_dot_AB(:, 1) = q_dot;
    h_SC = MOI * w;
    h_RW = rw_rot_axis * rw_J * rw_w;
    skew_w =[0 -w(3) w(2)
             w(3) 0 -w(1)
            -w(2) w(1) 0];
    w_dot = -MOI\(skew_w*h_SC-rw_torque+skew_w*h_RW);
    w_dot_AB(:, 2:3) = w_dot_AB(:, 1:2);
    w_dot_AB(:, 1) = w_dot;
    
    q = q + (q_dot_AB * AB) * dt;  
    q = q/norm(q);
    
    w = w + (w_dot_AB * AB) * dt;

    data_out( 1,i) = w(1);
    data_out( 2,i) = w(2);
    data_out( 3,i) = w(3);
    data_out( 4,i) = rw_w(1);
    data_out( 5,i) = rw_w(2);
    data_out( 6,i) = rw_w(3);
    data_out( 7,i) = rw_cmd(1);
    data_out( 8,i) = rw_cmd(2);
    data_out( 9,i) = rw_cmd(3);
    data_out(10,i) = rw_torque(1);
    data_out(11,i) = rw_torque(2);
    data_out(12,i) = rw_torque(3);
    data_out(13,i) = q(1);
    data_out(14,i) = q(2);
    data_out(15,i) = q(3);
    data_out(16,i) = q(4);
end

x = data_out(1:3,10000:end)';
 
t_final = 20*pi;
dt = 0.01;
t_clean = 0:dt:t_final;
n = length(t_clean);
w_1 = sin(t_clean);
w_2 = cos(t_clean);
w_3= sin(3*t_clean);
save bathymetry_real_clean.mat x

%plot things
figure, grid on, hold on
plot(t, (180/pi)*data_out(1:3,:))
xlabel('Time, s')
ylabel('Angular Rates, deg/s')
title('S/C Rotation Rates vs Time')
legend('X','Y','Z')

figure, grid on, hold on
plot(t, data_out(7:9,:))
title('RW Command')
xlabel('Time, s')
ylabel('cmd')
legend('X','Y','Z')

figure, grid on, hold on
plot(t, 100*data_out(4:6,:)/rw_max_omega)
title('RW Speed')
xlabel('Time, s')
ylabel('w, percent')
legend('X','Y','Z')

figure, grid on, hold on
plot(t, 10^3*data_out(10:12,:))
xlabel('Time, s')
ylabel('\tau, mNm')
title('RW Torque vs Time')

figure, grid on, hold on
plot(t, data_out(13:16, :))
plot(t, q_t(1) * ones(size(t)), 'k--')
plot(t, q_t(2) * ones(size(t)), 'k--')
plot(t, q_t(3) * ones(size(t)), 'k--')
plot(t, q_t(4) * ones(size(t)), 'k--')
xlabel('Time, s')
ylabel('q')
title('Attitude vs Time')
legend('qs','qx','qy','qz')