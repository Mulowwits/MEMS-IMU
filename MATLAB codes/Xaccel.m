addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

LOAD = readtable('Calibrated.csv','HeaderLines',0)
TIME_DATE_STRING = string(table2cell(LOAD(:,1)));
DATE_TIME = datetime(TIME_DATE_STRING);
% Obtain sensor data
 Ax = table2array(LOAD(:,2))/16384;
 Ay = table2array(LOAD(:,3))/16384;
 Az= table2array(LOAD(:,4))/16384;

   figure('Name', 'Calibrate');
   axis(1) = subplot(2,1,1);
   hold on
    plot(DATE_TIME,Ax,'b',DATE_TIME,Ay,'g',DATE_TIME,Az,'r','LineWidth',1.5)
    xlim([DATE_TIME(1) DATE_TIME(length(Ax))])
    grid on
    grid minor
    title('Linear Acceleration vs Time')
    ylabel('Measured Acceleration(g)')
    xlabel('Time')
    legend({'X coordinate','Y coordinate','Z coordinate'})
hold off
 Wx = table2array(LOAD(:,5));
 Wy = table2array(LOAD(:,6));
 Wz= table2array(LOAD(:,7));
   axis(1) = subplot(2,1,2);
   hold on
    plot(DATE_TIME,Wx/32.8,'b',DATE_TIME,Wy/32.8,'g',DATE_TIME,Wz/32.8,'r','LineWidth',1.5)
    xlim([DATE_TIME(1) DATE_TIME(length(Ax))])
    grid on
    grid minor
    title('Angular Acceleration vs Time')
    ylabel('Angular rate (deg/s)')
    xlabel('Time')
    legend({'X coordinate','Y coordinate','Z coordinate'})
     hold off
 Mx = table2array(LOAD(:,8));
 My = table2array(LOAD(:,9));
 Mz= table2array(LOAD(:,10));

  figure(3)
    plot(DATE_TIME,Mx,'b',DATE_TIME,My,'g',DATE_TIME,Mz,'r','LineWidth',1)
    xlim([DATE_TIME(1) DATE_TIME(length(Ax))])
    grid on
    grid minor
    title('Magnetometer reading')
    ylabel('Measured Magnetic field(LSB)')
    xlabel('Time')
    legend({'X coordinate','Y coordinate','Z coordinate'})
 %%%%%%%%%%%%%%%%%%%%%%%%%%%% Foward Path     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Gyroscope=[Wx/32.8,Wy/32.8,Wz/32.8]
Accelerometer=[Ax,Ay,Az]
Magnetometer=[Mx/0.16,My/0.16,Mz/0.16]

  AHRS = MadgwickAHRS('SamplePeriod',1/5, 'Beta', 0.2);
 %AHRS = MahonyAHRS('SamplePeriod',1/256, 'Kp', 0.5);

quaternion = zeros(length(DATE_TIME), 4);
for t = 1:length(DATE_TIME)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi)	% use conjugate for sensor frame relative to Earth and convert to degrees.
figure('Name', 'Attitude update');
axis(1) = subplot(2,1,1);
hold on
%figure('Name', 'Euler Angles Inward path');
hold on;
plot(DATE_TIME, euler(:,1), 'r','LineWidth',1.5);
plot(DATE_TIME, euler(:,2), 'g','LineWidth',1.5);
plot(DATE_TIME, euler(:,3), 'b','LineWidth',1.5);
title('Euler Angles Inward path');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi roll', '\theta pitch', '\psi yaw');
hold off;

%igure('Name', 'Attitude');
axis(1) = subplot(2,1,2);
hold on
plot(DATE_TIME,table2array(LOAD(:,14)),'b',DATE_TIME,table2array(LOAD(:,15)),'g',DATE_TIME,table2array(LOAD(:,16)),'m','LineWidth',1.5)
xlim([DATE_TIME(1) DATE_TIME(length(table2array(LOAD(:,14))))])
title('Euler angles Inward');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi roll', '\theta pitch', '\psi yaw');
hold off