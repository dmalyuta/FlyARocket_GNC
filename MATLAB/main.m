% This is the main file of the Falco-4 GNC simulation

clear;

% --------------------------------- SETUP --------------------------------------
% Here we set up the simulation parameters

global VALVE__MAX_THRUST; VALVE__MAX_THRUST=0.2; % [N]
global VALVE__SLEW_RATE; VALVE__SLEW_RATE=5; % [N], both going up and coming down
global CONTROL__TIME_STEP; CONTROL__TIME_STEP=1/50; % [s] control loop time interval (=1/frequency)
global CONTROL__START_TIME; CONTROL__START_TIME=1; % [s] time during simulation at which active control is turned on
global TIME__DROPOFF; TIME__DROPOFF=2; % [s] time during which valves can output 100% of VALVE__MAX_THRUST
global TIME__FULL; TIME__FULL=7; % [s] time at which no more gas in container, so valve thrust drops to 0
global DROPOFF_CONSTANT; DROPOFF_CONSTANT=1.5; % [s] time constant in dropoff characteristic, i.e. time in [s] after TIME__DROPOFF when valve max thrust reaches ~63.2% of VALVE__MAX_THRUST
global TIME_RCS_WORKED; TIME_RCS_WORKED=0; % [s] holds the time for which the RCS has been active
total_time=11; % [s] Simulation finish time

psi_0   =d2r(20);   % Initial yaw angle
wz_0    =d2r(0);    % Initial Z-body rate
theta_0 =d2r(-20);   % Initial pitch angle
wy_0    =d2r(0);    % Initial Y-body rate
phi_0   =d2r(0);    % Initial roll angle
wx_0    =d2r(0);   % Intiial X-body rate

global t_last; t_last=0; % The previous time that control was applied
max_timestep=CONTROL__TIME_STEP/10; % Upper bound for time step simulation

%*** RCS geometry
l=0.37403; % [m]Rocket length
d=0.005; % [m] Valve nozzle offset
r=0.076/2; % [m] Fuselage outer diameter

x_R1 = [l;-d;-r]; % Vector from c.o.m. to valve R1 nozzle
x_R2 = [l;r;d]; % Vector from c.o.m. to valve R2 nozzle
x_R3 = [l;d;r]; % Vector from c.o.m. to valve R3 nozzle
x_R4 = [l;-r;-d]; % Vector from c.o.m. to valve R4 nozzle

%*** Rocket parameters
I=[0.00206234	0.00000000	0.00000000
   0.00000000	0.36087211	0.00000000
   0.00000000	0.00000000	0.36087211];%diag([0.000240854442;0.062914314;0.062914314]); % [kg.m^2] Rocket inertia matrix in principal body axes

%------------------------------------ Noisy parameter estimation ------------------------------------
% Open the noise file
% This is a saved imu_log that was output when not moving the rocket, i.e.
% it contains only noise. DO NOT DELETE THIS FILE!
Noise_log = fopen('./logs/noise_log.txt','r');
Noise_data=textscan(Noise_log,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(Noise_log);

psi_noise=Noise_data{3};
theta_noise=Noise_data{4};
phi_noise=Noise_data{5};
psidot_noise=Noise_data{6};
thetadot_noise=Noise_data{7};
phidot_noise=Noise_data{8};

%*** Now we wish to estimate the average (mean) and covariance matrix on the noise of signals (psi,psidot), (theta,thetadot) and (phi,phidot)
% Averages of angles
mu_psi=mean(psi_noise);
mu_theta=mean(theta_noise);
mu_phi=mean(phi_noise);

% Averages of angular rates
mu_psidot=mean(psidot_noise);
mu_thetadot=mean(thetadot_noise);
mu_phidot=mean(phidot_noise);

% Covariance matrix for yaw-related values (angle and angular rate)
Cov_psi(1,1)=mean((psi_noise-mu_psi).*(psi_noise-mu_psi));
Cov_psi(1,2)=mean((psi_noise-mu_psi).*(psidot_noise-mu_psidot));
Cov_psi(2,1)=mean((psidot_noise-mu_psidot).*(psi_noise-mu_psi));
Cov_psi(2,2)=mean((psidot_noise-mu_psidot).*(psidot_noise-mu_psidot));

Cov_psi_cholesky=chol(Cov_psi);

% Covariance matrix for pitch-related values (angle and angular rate)
Cov_theta(1,1)=mean((theta_noise-mu_theta).*(theta_noise-mu_theta));
Cov_theta(1,2)=mean((theta_noise-mu_theta).*(thetadot_noise-mu_thetadot));
Cov_theta(2,1)=mean((thetadot_noise-mu_thetadot).*(theta_noise-mu_theta));
Cov_theta(2,2)=mean((thetadot_noise-mu_thetadot).*(thetadot_noise-mu_thetadot));

Cov_theta_cholesky=chol(Cov_theta);

% Covariance matrix for roll-related values (angle and angular rate)
Cov_phi(1,1)=mean((phi_noise-mu_phi).*(phi_noise-mu_phi));
Cov_phi(1,2)=mean((phi_noise-mu_phi).*(phidot_noise-mu_phidot));
Cov_phi(2,1)=mean((phidot_noise-mu_phidot).*(phi_noise-mu_phi));
Cov_phi(2,2)=mean((phidot_noise-mu_phidot).*(phidot_noise-mu_phidot));

Cov_phi_cholesky=chol(Cov_phi);

%------------------------------------ Kalman filter parameter setup ------------------------------------
global P_psi; P_psi = 1*eye(2); % Initial covariance matrix of the estimate
global x_psi; x_psi = [0;0];
global Q_psi; Q_psi = diag([0.01,100]); % Process noise covariance matrix
global R_psi; R_psi = 10; % Measurement noise covariance matrix

global P_psidot; P_psidot = 1*eye(2);
global x_psidot; x_psidot = [0;0];
global Q_psidot; Q_psidot = diag([200,200]);
global R_psidot; R_psidot = 5000;

global P_theta; P_theta = P_psi;
global x_theta; x_theta = [0;0];
global Q_theta; Q_theta = Q_psi;
global R_theta; R_theta = R_psi;

global P_thetadot; P_thetadot = P_psidot;
global x_thetadot; x_thetadot = [0;0];
global Q_thetadot; Q_thetadot = Q_psidot;
global R_thetadot; R_thetadot = R_psidot;

global P_phi; P_phi = P_psi;
global x_phi; x_phi = [0;0];
global Q_phi; Q_phi = Q_psi;
global R_phi; R_phi = R_psi;

global P_phidot; P_phidot = P_psidot;
global x_phidot; x_phidot = [0;0];
global Q_phidot; Q_phidot = Q_psidot;
global R_phidot; R_phidot = R_psidot;

%*** Initialize forces
global Fyaw; Fyaw=0;
global Fpitch; Fpitch=0;
global Mroll; Mroll=0;

global R1; R1=[0;0;0];
global R2; R2=[0;0;0];
global R3; R3=[0;0;0];
global R4; R4=[0;0;0];

psi_ref=0;
theta_ref=0;
wx_ref=0;

%*** Setup control logic coefficients
% Pitch control loop (PD control)
Fpitch_loop.K = 5; % Proportional term coefficient
Fpitch_loop.Td = 3; % Derivatice term coefficient

% Yaw control loop (PD control)
Fyaw_loop.K = 5; % Proportional term coefficient
Fyaw_loop.Td = 3; % Derivatice term coefficient

% Roll control loop (P control)
Mroll_loop.satur = 2*d*VALVE__MAX_THRUST;
Mroll_loop.control_range = 100*pi/180; % [rad/s]
Mroll_loop.K = Mroll_loop.satur/Mroll_loop.control_range; % Proportional term coefficient

%------------------------------- SIMULATION ------------------------------------
global data_log; data_log=[];
%*** Runge-Kutta integration of the rocket orientation evolution under control
options=odeset('MaxStep',max_timestep);
[t,x]=ode45(@rocket_dynamics,[0 total_time],[psi_0 wz_0 theta_0 wy_0 phi_0 wx_0],options,d,x_R1,x_R2,x_R3,x_R4,I,...
            Fpitch_loop,Fyaw_loop,Mroll_loop,psi_ref,theta_ref,wx_ref,total_time,...
            mu_psi,mu_theta,mu_phi,mu_psidot,mu_thetadot,mu_phidot,Cov_psi_cholesky,Cov_theta_cholesky,Cov_phi_cholesky);
       
% Give results meaningful names
psi=x(:,1);
wz=x(:,2);
theta=x(:,3);
wy=x(:,4);
phi=x(:,5);
wx=x(:,6);

c_time=data_log(:,1);
dt=data_log(:,2);
psi_imu=data_log(:,3);
theta_imu=data_log(:,4);
phi_imu=data_log(:,5);
psidot_imu=data_log(:,6);
thetadot_imu=data_log(:,7);
phidot_imu=data_log(:,8);
psi_filt=data_log(:,9);
theta_filt=data_log(:,10);
phi_filt=data_log(:,11);
psidot_filt=data_log(:,12);
thetadot_filt=data_log(:,13);
phidot_filt=data_log(:,14);
wx_filt=data_log(:,15);
wy_filt=data_log(:,16);
wz_filt=data_log(:,17);
Fpitch=data_log(:,18);
Fyaw=data_log(:,19);
Mroll=data_log(:,20);
R1=data_log(:,21);
R2=data_log(:,22);
R3=data_log(:,23);
R4=data_log(:,24);
% Deduce Euler angle rate of change from body rates
psidot=(wy.*sin(phi)+wz.*cos(phi))./cos(theta);
thetadot=wy.*cos(phi)-wz.*sin(phi);
phidot=wx+psidot.*sin(theta);
%--------------------------- RESULTS PROCESSING --------------------------------
filter_plot=figure(1);
figure(filter_plot); clf(filter_plot);
subplot(2,3,1);
hold on;
plot(c_time,r2d(psi_imu),'Color','red'); % psi_imu
plot(c_time,r2d(psi_filt),'Color','black'); % psi_filt
leg=legend('Raw yaw $\psi$','Filtered yaw $\psi_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,2);
hold on;
plot(c_time,r2d(theta_imu),'Color','red'); % theta_imu
plot(c_time,r2d(theta_filt),'Color','black'); % theta_filt
leg=legend('Raw pitch $\theta$','Filtered pitch $\theta_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,3);
hold on;
plot(c_time,r2d(phi_imu),'Color','red'); % phi_imu
plot(c_time,r2d(phi_filt),'Color','black'); % phi_filt
leg=legend('Raw roll $\phi$','Filtered roll $\phi_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,4);
hold on;
plot(c_time,r2d(psidot_imu),'Color','red'); % psidot_imu
plot(c_time,r2d(psidot_filt),'Color','black'); % psidot_filt
leg=legend('Raw yaw rate $\dot\psi$','Filtered yaw rate $\dot\psi_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,5);
hold on;
plot(c_time,r2d(thetadot_imu),'Color','red'); % thetadot_imu
plot(c_time,r2d(thetadot_filt),'Color','black'); % thetadot_filt
leg=legend('Raw pitch rate $\dot\theta$','Filtered pitch rate $\dot\theta_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,6);
hold on;
plot(c_time,r2d(phidot_imu),'Color','red'); % phidot_imu
plot(c_time,r2d(phidot_filt),'Color','black'); % phidot_filt
leg=legend('Raw roll rate $\dot\psi$','Filtered roll rate $\dot\phi_f$'); set(leg,'Interpreter', 'latex');

valve_thrust_plot=figure(2);
figure(valve_thrust_plot); clf(valve_thrust_plot);
subplot(4,1,1);
hold on;
plot(c_time,R1,'Color','black');
legend('Valve R1 thrust [N]');
subplot(4,1,2);
hold on;
plot(c_time,R2,'Color','black');
legend('Valve R2 thrust [N]');
subplot(4,1,3);
hold on;
plot(c_time,R3,'Color','black');
legend('Valve R3 thrust [N]');
subplot(4,1,4);
hold on;
plot(c_time,R4,'Color','black');
legend('Valve R4 thrust [N]');

attitude_plot=figure(3);
figure(attitude_plot); clf(attitude_plot);
subplot(2,1,1);
hold on;
plot(t,r2d(psi),'Color','black'); % Yaw angle
plot(t,r2d(theta),'Color','red'); % Pitch angle
plot(t,r2d(phi),'Color','blue'); % Roll angle
plot(t,r2d(psidot),'Color','green'); % Yaw rate
plot(t,r2d(thetadot),'Color','cyan'); % Pitch rate
plot(t,r2d(phidot),'Color','magenta'); % Roll rate
ylim([-20 20]); % Only interested in the +/- 20 [�] range as that's how bounded rocket angles are expected to be
leg=legend('Yaw $\psi$','Pitch $\theta$','Roll $\phi$','Yaw rate $\dot\psi$','Pitch rate $\dot\theta$','Roll rate $\dot\phi$'); set(leg,'Interpreter', 'latex');

subplot(2,2,3);
hold on;
plot(c_time,Fyaw,'Color','black'); % Control yaw force
plot(c_time,Fpitch,'Color','blue'); % Control pitch force
plot(c_time,ones(length(c_time),1)*VALVE__MAX_THRUST,'Color','red','LineStyle','--'); % Upper saturation limit
plot(c_time,-ones(length(c_time),1)*VALVE__MAX_THRUST,'Color','red','LineStyle','--'); % Lower saturation limit
leg=legend('Yaw force $F_{yaw}$ [N]','Pitch force $F_{yaw} [N]$'); set(leg,'Interpreter', 'latex');

subplot(2,2,4);
hold on;
plot(c_time,Mroll,'Color','blue'); % Control roll moment
plot(c_time,ones(length(c_time),1)*2*d*VALVE__MAX_THRUST,'Color','red','LineStyle','--'); % Upper saturation limit
plot(c_time,-ones(length(c_time),1)*2*d*VALVE__MAX_THRUST,'Color','red','LineStyle','--'); % Lower saturation limit
leg=legend('Roll moment $M_{roll}$ [N$\cdot$m]'); set(leg,'Interpreter', 'latex');