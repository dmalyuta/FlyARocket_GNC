% This file tests the Kalman filtering of the C program as well as the MATLAB
% code that immitates the C code processins for in-MATLAB software simulation of the
% rocket attitude control

clear;

%% Load the data from the C program

% Open the noise file
% This is a saved imu_log that was output when not moving the rocket, i.e.
% it contains only noise. DO NOT DELETE THIS FILE!
Noise_log = fopen('./logs/noise_log.txt','r');
Noise_data=textscan(Noise_log,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(Noise_log);

for ii=1:length(Noise_data{1}(1:end))
    time(ii,1)=Noise_data{1}(ii)/1000000;
    dt(ii,1)=Noise_data{2}(ii);
    psi_noise(ii,1)=Noise_data{3}(ii);
    theta_noise(ii,1)=Noise_data{4}(ii);
    phi_noise(ii,1)=Noise_data{5}(ii);
    psidot_noise(ii,1)=Noise_data{6}(ii);
    thetadot_noise(ii,1)=Noise_data{7}(ii);
    phidot_noise(ii,1)=Noise_data{8}(ii);
end

%% Parameter estimation

% We wish to find the noise average value and covariance matrix
% Assumption made : these value are the same for the 3 signals : yaw, pitch ( and roll (phi)

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

%% Generate noisy test signals

%*** Now we want to check visually that our calculation is sound
%*** For this let's create some period signal for each Euler angle, derive it for the angular rate
%    and subsequently add normally distributed noise generated given our mu_* and Cov_* values and
%    check out whether signal quality that we get is on-par with what comes from the IMU (which may be
%    check from the values loaded in from imu_log.txt at the beginning of this file in the "Load the data from
%    the C program" section

freq=1/30; % [Hz] Sinusoid frequency
ampli=20*pi/180; % [(rad)] Sinusoid amplitude
psi_perfect=ampli*(sin(time*2*pi*freq)+sin(time*2*pi*freq/2-pi/4));
theta_perfect=ampli*(sin(time*2*pi*freq)+sin(time*2*pi*freq*5-pi/6));
phi_perfect=ampli*(sin(time*2*pi*freq)+sin(time*2*pi*freq/5-pi/2));

psidot_perfect=ampli*(2*pi*freq*cos(time*2*pi*freq)+2*pi*freq/2*cos(time*2*pi*freq/2-pi/4));
thetadot_perfect=ampli*(2*pi*freq*cos(time*2*pi*freq)+2*pi*freq*5*cos(time*2*pi*freq*5-pi/6));
phidot_perfect=ampli*(2*pi*freq*cos(time*2*pi*freq)+2*pi*freq/5*cos(time*2*pi*freq/5-pi/2));

% Now generate the noisy signals
psi_noisy=zeros(length(time),1);
psidot_noisy=zeros(length(time),1);
theta_noisy=zeros(length(time),1);
thetadot_noisy=zeros(length(time),1);
phi_noisy=zeros(length(time),1);
phidot_noisy=zeros(length(time),1);
for ii=1:length(time)
    noise_on_psi=[mu_psi mu_psidot]+randn(1,2)*Cov_psi_cholesky; % Bivariate normal distribution of noise for the (psi,psidot) signal
    noise_on_theta=[mu_theta mu_thetadot]+randn(1,2)*Cov_theta_cholesky; % Bivariate normal distribution of noise for the (theta,thetadot) signal
    noise_on_phi=[mu_phi mu_phidot]+randn(1,2)*Cov_phi_cholesky; % Bivariate normal distribution of noise for the (phi,phidot) signal
    
    psi_noisy(ii)=psi_perfect(ii)+noise_on_psi(1);
    psidot_noisy(ii)=psidot_perfect(ii)+noise_on_psi(2);
    theta_noisy(ii)=theta_perfect(ii)+noise_on_theta(1);
    thetadot_noisy(ii)=thetadot_perfect(ii)+noise_on_theta(2);
    phi_noisy(ii)=phi_perfect(ii)+noise_on_phi(1);
    phidot_noisy(ii)=phidot_perfect(ii)+noise_on_phi(2);
end

%% Apply Kalman filter

%*** We want to test our Kalman filter on the generated noisy test signals
% Setup Kalman filter parameters
P_psi = 1*eye(2); % Initial covariance matrix of the estimate
x_psi = [0;0];
Q_psi = diag([0.01,100]); % State error covariance matrix
R_psi = 10; % Output covariance matrix

P_psidot = 1*eye(2); % Initial covariance matrix of the estimate
x_psidot = [0;0];
Q_psidot = diag([200,200]); % State error covariance matrix
R_psidot = 5000; % Output covariance matrix

P_theta = P_psi;
x_theta = x_psi;
Q_theta = Q_psi;
R_theta = R_psi;

P_thetadot = P_psidot;
x_thetadot = x_psidot;
Q_thetadot = Q_psidot;
R_thetadot = R_psidot;

P_phi = P_psi;
x_phi = x_psi;
Q_phi = Q_psi;
R_phi = R_psi;

P_phidot = P_psidot;
x_phidot = x_psidot;
Q_phidot = Q_psidot;
R_phidot = R_psidot;

% Now filter
psi_filt=zeros(length(time),1);
psidot_filt=zeros(length(time),1);
theta_filt=zeros(length(time),1);
thetadot_filt=zeros(length(time),1);
phi_filt=zeros(length(time),1);
phidot_filt=zeros(length(time),1);
for ii=1:length(time)    
    [x_psi,P_psi] = kalmanFnc(x_psi,P_psi,psi_noisy(ii),Q_psi,R_psi,dt(ii));
    [x_psidot,P_psidot] = kalmanFnc(x_psidot,P_psidot,psidot_noisy(ii),Q_psidot,R_psidot,dt(ii));
    [x_theta,P_theta] = kalmanFnc(x_theta,P_theta,theta_noisy(ii),Q_theta,R_theta,dt(ii));
    [x_thetadot,P_thetadot] = kalmanFnc(x_thetadot,P_thetadot,thetadot_noisy(ii),Q_thetadot,R_thetadot,dt(ii));
    [x_phi,P_phi] = kalmanFnc(x_phi,P_phi,phi_noisy(ii),Q_phi,R_phi,dt(ii));
    [x_phidot,P_phidot] = kalmanFnc(x_phidot,P_phidot,phidot_noisy(ii),Q_phidot,R_phidot,dt(ii));
    
    psi_filt(ii)=x_psi(1);
    psidot_filt(ii)=x_psidot(1);
    theta_filt(ii)=x_theta(1);
    thetadot_filt(ii)=x_thetadot(1);
    phi_filt(ii)=x_phi(1);
    phidot_filt(ii)=x_phidot(1);
end

%% Plot results

filter_check=figure(1);
figure(filter_check); clf(filter_check);

subplot(2,3,1); hold on;
plot(time,psi_perfect,'Color','black');
plot(time,psi_noisy,'Color','red');
plot(time,psi_filt,'Color','blue');
leg=legend('Perfect yaw $\psi$','Noisy yaw $\psi_n$','Filtered yaw $\psi_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,2); hold on;
plot(time,theta_perfect,'Color','black');
plot(time,theta_noisy,'Color','red');
plot(time,theta_filt,'Color','blue');
leg=legend('Perfect pitch $\theta$','Noisy pitch $\theta_n$','Filtered pitch $\theta_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,3); hold on;
plot(time,phi_perfect,'Color','black');
plot(time,phi_noisy,'Color','red');
plot(time,phi_filt,'Color','blue');
leg=legend('Perfect roll $\phi$','Noisy roll $\phi_n$','Filtered roll $\phi_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,4); hold on;
plot(time,psidot_perfect,'Color','black');
plot(time,psidot_noisy,'Color','red');
plot(time,psidot_filt,'Color','blue');
leg=legend('Perfect yaw rate $\dot\psi$','Noisy yaw rate $\dot\psi_n$','Filtered yaw rate $\dot\psi_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,5); hold on;
plot(time,thetadot_perfect,'Color','black');
plot(time,thetadot_noisy,'Color','red');
plot(time,thetadot_filt,'Color','blue');
leg=legend('Perfect pitch rate $\dot\theta$','Noisy pitch rate $\dot\theta_n$','Filtered pitch rate $\dot\theta_f$'); set(leg,'Interpreter', 'latex');

subplot(2,3,6); hold on;
plot(time,phidot_perfect,'Color','black');
plot(time,phidot_noisy,'Color','red');
plot(time,phidot_filt,'Color','blue');
leg=legend('Perfect roll rate $\dot\phi$','Noisy roll rate $\dot\phi_n$','Filtered roll rate $\dot\phi_f$'); set(leg,'Interpreter', 'latex');