addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

LOAD = readtable('moveY.csv','HeaderLines',0)
second_data=readtable('moveY2.csv','HeaderLines',0);
average_of_two=readtable('combined1.csv','HeaderLines',0);
TIME_DATE_STRING = string(table2cell(LOAD(:,1)));
DATE_TIME = datetime(TIME_DATE_STRING);
TIME_DATE_STRING2 = string(table2cell(second_data(:,1)));
DATE_TIME2 = datetime(TIME_DATE_STRING2);
TIME_DATE_STRING3 = string(table2cell(average_of_two(:,1)));
DATE_TIME3 = datetime(TIME_DATE_STRING3);
%%%%%%%%%%%%%% Obtain sensor data %%%%%%%%%%%%%%%%%%%%%
Ax = table2array(LOAD(:,2));
Ay = table2array(LOAD(:,3));
Az= table2array(LOAD(:,4));
N=4;
fs=100;
fc=0.4 ;
[B,A]=butter(N,2*fc/fs,'high');

%%%%%%%%%%%%%%%%%%%%% Calibration raw data %%%%%%%%%%%%%%%%%%%%%%%%
X_calibrate=readtable('CalibrateX.csv','HeaderLines',0);
Y_calibrate=readtable('CalibrateY.csv','HeaderLines',0);
Z_calibrate=readtable('CalibrateZ.csv','HeaderLines',0);
Gyro_calibrate=readtable('Gyrocalibrate.csv','HeaderLines',0);
gravity=9.81;
% % Ay_2 = table2array(second_data(:,3));
% % Az_2= table2array(second_data(:,4));

Ax_min=min(table2array(X_calibrate(:,2))/16384)
Ax_max=max(table2array(X_calibrate(:,2))/16384)
Ay_min=min(table2array(Y_calibrate(:,3))/16384)
Ay_max=max(table2array(Y_calibrate(:,3))/16384)
Az_min=min(table2array(Z_calibrate(:,4))/16384)
Az_max=max(table2array(Z_calibrate(:,4))/16384)

AccelOffset_Matrix=[((1+Ax_min)+(1-Ax_max))/2;((1+Ay_min)+(1-Ay_max))/2;((1+Az_min)-(1-Az_max))/2]%
AccelScale_Offset=[gravity/(Ax_max-AccelOffset_Matrix(1));gravity/(Ay_max-AccelOffset_Matrix(2));gravity/(Az_max-AccelOffset_Matrix(3))]
Angularx=table2array(Gyro_calibrate(:,5));
Angulary=table2array(Gyro_calibrate(:,6));
Angularz = table2array(Gyro_calibrate(:,7));
GO_X=mean(Angularx)
GO_Y=mean(Angulary)
G0_Z=mean(Angularz)


%% End of script
% Accelx= filter(B,A,Ax*9.81/16384)%(Ax/16384)
% Accely= filter(B,A,Ay*9.81/16384)%(Ay/16384)
% Accelz=filter(B,A,Az*9.81/16384)%(Az/16384)
% Ax_2 = table2array(second_data(:,2));
% Ay_2 = table2array(second_data(:,3));
% Az_2= table2array(second_data(:,4));
% Accelx_2=filter(B,A,Ax_2*9.81/16384)%(Ax_2/16384)
% Accely_2=filter(B,A,Ay_2*9.81/16384)%(Ay_2/16384)
% Accelz_2=filter(B,A,Az_2*9.81/16384)%(Az_2/16384)

% N=4;
% fs=length(Accelx)/22;
% fc=0.05;
% [B,A]=butter(N,2*fc/fs,'high');
% 
% Accelx22= filter(B,A,Accely*9.8)%(Ax/16384)


Accelx= (Ax/16384);
Accely= (Ay/16384);
Accelz=(Az/16384)+0.08;
Ax_2 = table2array(second_data(:,2));
Ay_2 = table2array(second_data(:,3));
Az_2= table2array(second_data(:,4));
Accelx_2=(Ax_2/16384);
Accely_2=(Ay_2/16384);
Accelz_2=(Az_2/16384)+0.08;
Ax_3 = table2array(second_data(:,2));
Ay_3 = table2array(second_data(:,3));
Az_3= table2array(second_data(:,4));
Accelx_3=(Ax_3/16384);
Accely_3=(Ay_3/16384);
Accelz_3=(Az_3/16384)+0.08;
X_notFiltered=(Ax*9.81/16384);
Y_notFiltered=(Ay*9.81/16384);
Z_notFiltered=(Az*9.81/16384);
Wx = table2array(LOAD(:,5));
Wy = table2array(LOAD(:,6));
Wz= table2array(LOAD(:,7));
Mx = table2array(LOAD(:,8));
My = table2array(LOAD(:,9));
Mz= table2array(LOAD(:,10));

Wx_o = table2array(second_data(:,5));
Wy_o = table2array(second_data(:,6));
Wz_o= table2array(second_data(:,7));
Mx_o = table2array(second_data(:,8));
My_o = table2array(second_data(:,9));
Mz_o= table2array(second_data(:,10));

Velocity=cumtrapz(0.2,(Accelz*9.81));
distance =cumtrapz(0.2,Velocity);

Uniform_time=table2array(LOAD(:,11));
Sample_time = unique(Uniform_time)
AVG_Xaccel = length(Sample_time);
AVG_Yaccel = length(Sample_time);
Heading_in=table2array(LOAD(:,17))
AVG_Zaccel= length(Sample_time);
AVG_Heading_In=length(Sample_time);
for n = 1:length(Sample_time)
    occurance = find (Uniform_time == Sample_time(n));
    AVG_Xaccel(n,:) = mean(Accelx(occurance));
    AVG_Yaccel(n,:) = mean(Accely(occurance));
    AVG_Zaccel(n,:) = mean(Accelz(occurance));
    AVG_Heading_I(n,:) = mean(Heading_in(occurance));
end

Velocity_x=cumtrapz(0.2,(AVG_Xaccel));
Velocity_y=cumtrapz(1,((AVG_Yaccel)*9.8));
Velocity_z=cumtrapz(0.2,(AVG_Zaccel*9.8));

distance_x =cumtrapz(0.2,Velocity_x);
distance_z =cumtrapz(0.2,Velocity_z);
distance_y=cumtrapz(1,Velocity_y);

figure(4)
plot(DATE_TIME,Accelx,'b',DATE_TIME,Accely,'r',DATE_TIME,Accelz,'g','LineWidth',1.5)
%xlim([DATE_TIME(1) DATE_TIME(length(Accelx))])
grid on
grid minor
title('Linear Acceleration vs Time')
ylabel('Measured Acceleration(g)')
xlabel('Time')
legend({'X coordinate','Y coordinate ','Z coordinate'})


Gyroscope=[Wx,Wy,Wz]
Accelerometer=[Accelx,Accely,Accelz]
Magnetometer=[Mx,My,Mz]
% Process sensor data through algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


 Uniform_time2=table2array(second_data(:,11));
    Sample_time2 = unique(Uniform_time2);
    AVG_Xaccel2 = length(Sample_time2);
    AVG_Yaccel2 = length(Sample_time2);
    AVG_Zaccel2= length(Sample_time2);

    for n = 1:length(Sample_time2)
        occurance2 = find (Uniform_time2 == Sample_time2(n));
        AVG_Xaccel2(n,:) = mean(Accelx_2(occurance2));
        AVG_Yaccel2(n,:) = mean(Accely_2(occurance2));
        AVG_Zaccel2(n,:) = mean(Accelz_2(occurance2));
    end
Velocity_x2=cumtrapz(0.2,(AVG_Xaccel2));
Velocity_y2=cumtrapz(1,(AVG_Yaccel2)*9.8);
Velocity_z2=cumtrapz(0.02,AVG_Zaccel2);

distance_x2 =cumtrapz(0.2,Velocity_x2);
distance_z2 =cumtrapz(0.2,Velocity_z2);
distance_y2=cumtrapz(1,Velocity_y2);

%%%%%%%%%%%%%%%%%%%%%%%%%% average %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Uniform_time3=table2array(average_of_two(:,11));
    Sample_time3 = unique(Uniform_time3);
    AVG_Xaccel3 = length(Sample_time3);
    AVG_Yaccel3 = length(Sample_time3);
    AVG_Zaccel3= length(Sample_time3);

    for n = 1:length(Sample_time3)
        occurance3 = find (Uniform_time3 == Sample_time3(n));
        AVG_Xaccel3(n,:) = mean(Accelx_3(occurance2));
        AVG_Yaccel3(n,:) = mean(Accely_3(occurance2));
        AVG_Zaccel3(n,:) = mean(Accelz_3(occurance2));
    end
Velocity_x3=cumtrapz(0.2,(AVG_Xaccel3));
Velocity_y3=cumtrapz(1,(AVG_Yaccel3)*9.8);
Velocity_z3=cumtrapz(0.2,AVG_Zaccel3);

distance_x3 =cumtrapz(0.2,Velocity_x3);
distance_z3 =cumtrapz(0.2,Velocity_z3);
distance_y3=cumtrapz(1,Velocity_y3);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(5)
plot(Sample_time,AVG_Xaccel,'c',Sample_time,AVG_Yaccel,'m',Sample_time,AVG_Zaccel,'k',Sample_time2,AVG_Xaccel2,'b',Sample_time2,AVG_Yaccel2,'r',Sample_time2,AVG_Zaccel2,'g','LineWidth',1.5)
%xlim([DATE_TIME(1) DATE_TIME(length(Accelx))])
grid on
grid minor
title('Linear Acceleration vs Time')
ylabel('Measured Acceleration(g)')
xlabel('Time(s)')
legend({'X inward','Y inward ','Z inward','X outward','Y outward ','Z outward'})

 inward_dist=acc2disp(AVG_Xaccel,1)
 outward_dist=acc2disp(AVG_Xaccel2,1)
 average_path=acc2disp(AVG_Xaccel3,1)

figure(11)
plot(Sample_time,distance_y*1000,'b','LineWidth',1.5) 
hold on
plot(Sample_time2,distance_y2*1000,'k','LineWidth',1.5)
plot(Sample_time3,distance_y3*1000,'r','LineWidth',1.5)
hold off
% xlim([time_x(1) time_x(length(distance_x))])
grid on
grid minor
title('Distance vs time')
ylabel('Measured distance(mm)')
xlabel('Time(s)')
legend({'Inward','Outward ','Average'})

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Foward Path     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Gyroscope=[Wx,Wy,Wz]
Accelerometer=[Accelx,Accely,Accelz]
Magnetometer=[Mx,My,Mz]

  AHRS = MadgwickAHRS('SamplePeriod',1/256, 'Beta', 0.1);
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
title('Euler Angles Forward path');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi roll', '\theta pitch', '\psi yaw');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%% Backward path %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Gyroscope_O=[Wx_o,Wy_o,Wz_o]
Accelerometer_O=[Accelx_2,Accely_2,Accelz_2]
Magnetometer_O=[Mx_o,My_o,Mz_o]

  AHRS_O = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);
 %AHRS_O = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion_O = zeros(length(DATE_TIME2), 4);
for t = 1:length(DATE_TIME2)
    AHRS_O.Update(Gyroscope_O(t,:) * (pi/180), Accelerometer_O(t,:), Magnetometer_O(t,:));	% gyroscope units must be radians
    quaternion_O(t, :) = AHRS_O.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler_O = quatern2euler(quaternConj(quaternion_O)) * (180/pi)	% use conjugate for sensor frame relative to Earth and convert to degrees.

%figure('Name', 'Euler Angles Outward path');
axis(1) = subplot(2,1,2);
hold on;
plot(DATE_TIME2, euler_O(:,1), 'r','LineWidth',1.5);
plot(DATE_TIME2, euler_O(:,2), 'g','LineWidth',1.5);
plot(DATE_TIME2, euler_O(:,3), 'b','LineWidth',1.5);
title('Euler Angles Backward path');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi roll', '\theta pitch', '\psi yaw');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


  %AHRS_O = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);
 AHRS_OM = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion_OM = zeros(length(DATE_TIME2), 4);
for t = 1:length(DATE_TIME2)
    AHRS_OM.Update(Gyroscope_O(t,:) * (pi/180), Accelerometer_O(t,:), Magnetometer_O(t,:));	% gyroscope units must be radians
    quaternion_OM(t, :) = AHRS_OM.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler_OM = quatern2euler(quaternConj(quaternion_OM)) * (180/pi)	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles Outward path');
axis(1) = subplot(2,1,1);
hold on;
plot(DATE_TIME2, euler_OM(:,1), 'r','LineWidth',1.5);
plot(DATE_TIME2, euler_OM(:,2), 'g','LineWidth',1.5);
plot(DATE_TIME2, euler_OM(:,3), 'b','LineWidth',1.5);
title('Euler Angles Outward path');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi roll', '\theta pitch', '\psi yaw');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



figure('Name', 'Attitude');
axis(1) = subplot(2,1,1);
hold on
plot(DATE_TIME,table2array(LOAD(:,14)),'b',DATE_TIME,table2array(LOAD(:,15)),'g',DATE_TIME,table2array(LOAD(:,16)),'m','LineWidth',1.5)
xlim([DATE_TIME(1) DATE_TIME(length(table2array(LOAD(:,14))))])
title('Euler angles Forward');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi roll', '\theta pitch', '\psi yaw');
hold off
axis(2) = subplot(2,1,2);
hold on
plot(DATE_TIME2,table2array(second_data(:,14)),'r',DATE_TIME2,table2array(second_data(:,15)),'c',DATE_TIME2,table2array(second_data(:,16)),'k','LineWidth',1.5)
xlim([DATE_TIME2(1) DATE_TIME2(length(table2array(second_data(:,14))))])
title('Euler angles Backward');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi roll', '\theta pitch', '\psi yaw','\phi roll', '\theta pitch', '\psi yaw');
hold off

% N=4;
% fs=length(Accelx)/22;
% fc=0.05;
% [B,A]=butter(N,2*fc/fs,'high');
% 
% Accelx22= filter(B,A,Accely*9.8)%(Ax/16384)
% 
% disp=acc2disp(Accelx22,0.2);
% %
% figure(5)
% plot(DATE_TIME,disp*1000)


%%%%%%%%%%%%%%%%% Apply filter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function disp_time_data = acc2disp(acc_time_data,dt)
% acc_time_data should be acceleration amplitudes evenly spaced in time
% dt in units of seconds per sample
N1 = length(acc_time_data);
  N = 2^nextpow2(N1);
  if N > N1
      acc_time_data(N1+1:N) = 0; % pad array with 0's
  end
  df = 1 / (N*dt); % frequency increment
  Nyq = 1 / (2*dt); % Nyquist frequency
  acc_freq_data = fftshift(fft(acc_time_data));
  disp_freq_data = zeros(size(acc_freq_data));
  f = -Nyq:df:Nyq-df; % I think this is how fftshift organizes it
  for i = 1 : N
      if f(i) ~= 0
          disp_freq_data(i) = acc_freq_data(i)/(2*pi*f(i)*sqrt(-1))^2;
      else
          disp_freq_data(i) = 0;
      end
  end
  disp_time_data = ifft(ifftshift(disp_freq_data));    
  disp_time_data = disp_time_data(1:N1);
% return
end

