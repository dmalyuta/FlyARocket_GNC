% This file analyzes the logged data by the Falco-4 flight software

%% COnfiguration parameters

VALVE__MAX_THRUST=0.5; % [N]

%% imu_log analysis

imu_log = fopen('./logs/imu_log.txt','r');
imu_data = textscan(imu_log,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(imu_log);

% Give loaded data meaningful names
time_imu_glob=imu_data{1}/1000000; % [s]
dt=imu_data{2};
psi_save=imu_data{3};
theta_save=imu_data{4};
phi_save=imu_data{5};
psi_dot=imu_data{6};
theta_dot=imu_data{7};
phi_dot=imu_data{8};
psi_filt=imu_data{9};
theta_filt=imu_data{10};
phi_filt=imu_data{11};
psi_dot_filt=imu_data{12};
theta_dot_filt=imu_data{13};
phi_dot_filt=imu_data{14};
wx=imu_data{15};
wy=imu_data{16};
wz=imu_data{17};
accelX_save=imu_data{18};
accelY_save=imu_data{19};
accelZ_save=imu_data{20};

%*** Plot data
% angle_plot plots the IMU raw and filtered angles and their derivatives
angle_plot=figure(1);
figure(angle_plot); clf(angle_plot);
subplot(2,3,1);
hold on;
plot(time_imu_glob,r2d(psi_save),'Color','red');
plot(time_imu_glob,r2d(psi_filt),'Color','black');
xlabel('Time [s]');
leg=legend('Raw yaw $\psi$ [deg]','Filtered yaw $\psi_f$ [deg]'); set(leg,'Interpreter', 'latex');

subplot(2,3,2);
hold on;
plot(time_imu_glob,r2d(theta_save),'Color','red');
plot(time_imu_glob,r2d(theta_filt),'Color','black');
xlabel('Time [s]');
leg=legend('Raw pitch $\theta$ [deg]','Filtered pitch $\theta_f$ [deg]'); set(leg,'Interpreter', 'latex');

subplot(2,3,3);
hold on;
plot(time_imu_glob,r2d(phi_save),'Color','red');
plot(time_imu_glob,r2d(phi_filt),'Color','black');
xlabel('Time [s]');
leg=legend('Raw roll $\phi$ [deg]','Filtered roll $\phi_f$ [deg]'); set(leg,'Interpreter', 'latex');

subplot(2,3,4);
hold on;
plot(time_imu_glob,r2d(psi_dot),'Color','red');
plot(time_imu_glob,r2d(psi_dot_filt),'Color','black');
xlabel('Time [s]');
leg=legend('Raw yaw rate $\dot\psi$ [deg/s]','Filtered yaw rate $\dot\psi_f$ [deg/s]'); set(leg,'Interpreter', 'latex');

subplot(2,3,5);
hold on;
plot(time_imu_glob,r2d(theta_dot),'Color','red');
plot(time_imu_glob,r2d(theta_dot_filt),'Color','black');
xlabel('Time [s]');
leg=legend('Raw pitch rate $\dot\theta$ [deg/s]','Filtered pitch rate $\dot\theta_f$ [deg/s]'); set(leg,'Interpreter', 'latex');

subplot(2,3,6);
hold on;
plot(time_imu_glob,r2d(phi_dot),'Color','red');
plot(time_imu_glob,r2d(phi_dot_filt),'Color','black');
xlabel('Time [s]');
leg=legend('Raw roll rate $\dot\psi$ [deg/s]','Filtered roll rate $\dot\phi_f$ [deg/s]'); set(leg,'Interpreter', 'latex');

% rate_plot shows the calculated body rates (i.e. rates of rotation about the body axes X, Y and Z attached to the rocket)
rate_plot=figure(2);
figure(rate_plot); clf(rate_plot);
subplot(3,1,1);
plot(time_imu_glob,r2d(wx),'Color','black');
xlabel('Time [s]');
leg=legend('X-body rate $\omega_x$ [deg/s]'); set(leg,'Interpreter', 'latex');

subplot(3,1,2);
plot(time_imu_glob,r2d(wy),'Color','black');
xlabel('Time [s]');
leg=legend('Y-body rate $\omega_y$ [deg/s]'); set(leg,'Interpreter', 'latex');

subplot(3,1,3);
plot(time_imu_glob,r2d(wz),'Color','black');
xlabel('Time [s]');
leg=legend('Z-body rate $\omega_z$ [deg/s]'); set(leg,'Interpreter', 'latex');

% accel_plot shows the acceleration from the ADXL345 triple-axis MEMS accelerometer that sits on the IMU
accel_plot=figure(3);
figure(accel_plot); clf(accel_plot);
subplot(3,1,1);
plot(time_imu_glob,accelX_save,'Color','black');
xlabel('Time [s]');
leg=legend('X acceleration $a_X$ [m/s$^2$]'); set(leg,'Interpreter', 'latex');

subplot(3,1,2);
plot(time_imu_glob,accelY_save,'Color','black');
xlabel('Time [s]');
leg=legend('Y-acceleration $a_Y$ [m/s$^2$]'); set(leg,'Interpreter', 'latex');

subplot(3,1,3);
plot(time_imu_glob,accelZ_save,'Color','black');
xlabel('Time [s]');
leg=legend('Z acceleration $a_Z$ [m/s$^2$]'); set(leg,'Interpreter', 'latex');

%% control_log analysis

control_log = fopen('./logs/control_log.txt','r');
control_data = textscan(control_log,'%f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(control_log);

time_control_glob=control_data{1}/1000000; % [s]
control_time=control_data{2};
Fpitch=control_data{3};
Fyaw=control_data{4};
Mroll=control_data{5};
R1=control_data{6};
R2=control_data{7};
R3=control_data{8};
R4=control_data{9};
PWM1=control_data{10};
PWM2=control_data{11};
PWM3=control_data{12};
PWM4=control_data{13};

% valve_plot shows the individual valve thrusts and the corresponding PWM input
valve_plot=figure(4);
figure(valve_plot); clf(valve_plot);
subplot(4,2,1);
hold on;
plot(time_control_glob,R1,'Color','black');
xlabel('Time [s]');
legend('Valve R1 thrust [N]');
subplot(4,2,2);
plot(time_control_glob,PWM1,'Color','black');
xlabel('Time [s]');
legend('Valve R1 PWM');
subplot(4,2,3);
plot(time_control_glob,R2,'Color','black');
xlabel('Time [s]');
legend('Valve R2 thrust [N]');
subplot(4,2,4);
plot(time_control_glob,PWM2,'Color','black');
xlabel('Time [s]');
legend('Valve R2 PWM');
subplot(4,2,5);
plot(time_control_glob,R3,'Color','black');
xlabel('Time [s]');
legend('Valve R3 thrust [N]');
subplot(4,2,6);
plot(time_control_glob,PWM3,'Color','black');
xlabel('Time [s]');
legend('Valve R3 PWM');
subplot(4,2,7);
plot(time_control_glob,R4,'Color','black');
xlabel('Time [s]');
legend('Valve R4 thrust [N]');
subplot(4,2,8);
plot(time_control_glob,PWM4,'Color','black');
xlabel('Time [s]');
legend('Valve R4 PWM');

% force_plot shows the rocket angles and how Fpitch, Fyaw and Mroll respond to them changing
start_idx=1;
end_idx=1;
for ii=1:length(time_imu_glob) % Sync the control data with the angle data
    if (time_imu_glob(ii)>=time_control_glob(1) && start_idx==1)
        start_idx=ii;
    end
    if (time_imu_glob(ii)>=time_control_glob(end))
        end_idx=ii;
        break;
    end
end

force_plot=figure(5);
figure(force_plot); clf(force_plot);
subplot(2,1,1);
hold on;
plot(time_imu_glob(start_idx:end_idx),r2d(psi_filt(start_idx:end_idx)),'Color','black'); % Yaw angle
plot(time_imu_glob(start_idx:end_idx),r2d(theta_filt(start_idx:end_idx)),'Color','red'); % Pitch angle
plot(time_imu_glob(start_idx:end_idx),r2d(phi_filt(start_idx:end_idx)),'Color','blue'); % Roll angle
%plot(time_imu_glob(start_idx:end_idx),r2d(psi_dot_filt(start_idx:end_idx)),'Color','green'); % Yaw rate
%plot(time_imu_glob(start_idx:end_idx),r2d(theta_dot_filt(start_idx:end_idx)),'Color','cyan'); % Pitch rate
%plot(time_imu_glob(start_idx:end_idx),r2d(phi_dot_filt(start_idx:end_idx)),'Color','magenta'); % Roll rate
xlabel('Time [s]');
leg=legend('Filtered yaw $\psi_f$','Filtered pitch $\theta_f$','Filtered roll $\phi_f$');%,'Filtered yaw rate $\dot\psi_f$','Filtered pitch rate $\dot\theta_f$','Filtered roll rate $\dot\phi_f$');
set(leg,'Interpreter', 'latex');

subplot(2,2,3);
hold on;
plot(time_control_glob,Fyaw,'Color','black'); % Control yaw force
plot(time_control_glob,Fpitch,'Color','blue'); % Control pitch force
plot(time_control_glob,ones(length(time_control_glob),1)*VALVE__MAX_THRUST,'Color','red','LineStyle','--'); % Upper saturation limit
plot(time_control_glob,-ones(length(time_control_glob),1)*VALVE__MAX_THRUST,'Color','red','LineStyle','--'); % Lower saturation limit
xlabel('Time [s]');
leg=legend('Yaw force $F_{yaw}$ [N]','Pitch force $F_{yaw} [N]$'); set(leg,'Interpreter', 'latex');

subplot(2,2,4);
hold on;
plot(time_control_glob,Mroll,'Color','blue'); % Control roll moment
plot(time_control_glob,ones(length(time_control_glob),1)*2*d*VALVE__MAX_THRUST,'Color','red','LineStyle','--'); % Upper saturation limit
plot(time_control_glob,-ones(length(time_control_glob),1)*2*d*VALVE__MAX_THRUST,'Color','red','LineStyle','--'); % Lower saturation limit
xlabel('Time [s]');
leg=legend('Roll moment $M_{roll}$ [N$\cdot$m]'); set(leg,'Interpreter', 'latex');

%% pressure_log analysis

pressure_log = fopen('./logs/pressure_log.txt','r');
pressure_data = textscan(pressure_log,'%f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(pressure_log);

time_pressure_glob=pressure_data{1};
radial_status=pressure_data{2};
radial_pressure=pressure_data{3};
radial_temperature=pressure_data{4};
axial_state=pressure_data{5};
axial_pressure=pressure_data{6};
axial_temperature=pressure_data{7};

% honeywell_plot shows the collected temperatures and pressures from the honeywell HSC sensors
honeywell_plot=figure(6);
figure(honeywell_plot); clf(honeywell_plot);
subplot(2,2,1);
plot(time_pressure_glob,radial_pressure,'Color','black');
xlabel('Time [s]');
leg=legend('Radial pressure $p_{rad}$ [mbar]'); set(leg,'Interpreter', 'latex');
subplot(2,2,3);
plot(time_pressure_glob,radial_temperature,'Color','black');
xlabel('Time [s]');
leg=legend('Radial temperature $T_{rad}$ [degC]'); set(leg,'Interpreter', 'latex');
subplot(2,2,2);
plot(time_pressure_glob,axial_pressure,'Color','black');
xlabel('Time [s]');
leg=legend('Axial pressure $p_{ax}$ [mbar]'); set(leg,'Interpreter', 'latex');
subplot(2,2,4);
plot(time_pressure_glob,axial_pressure,'Color','black');
xlabel('Time [s]');
leg=legend('Axial temperature $T_{ax}$ [degC]'); set(leg,'Interpreter', 'latex');