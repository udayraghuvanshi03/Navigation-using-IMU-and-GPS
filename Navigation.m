file1="/home/uday/catkin_ws/src/Analysis_LAB4/Circles.bag";
file2="/home/uday/catkin_ws/src/Analysis_LAB4/moving.bag";
file3="/home/uday/catkin_ws/src/Analysis_LAB4/stationary.bag";
circles_bag=rosbag(file1);
moving_bag=rosbag(file2);
stationary_bag=rosbag(file3);
baginfo1=rosbag('info',file1);
display(circles_bag);

bsel1=select(circles_bag,'Topic','/imutopic');
bsel2=select(moving_bag,'Topic','/imutopic');
bsel3=select(moving_bag,'Topic','/gpstopic');
bsel4=select(stationary_bag,'Topic','/imutopic');
msgs1=readMessages(bsel1,'DataFormat','struct');
msgs2=readMessages(bsel2,'DataFormat','struct');
msgs3=readMessages(bsel3,'DataFormat','struct');
msgs4=readMessages(bsel4,'DataFormat','struct');
display(msgs1{1});

% IMU DATA WHILE MOVING
orientation_x=cellfun(@(m) double(m.Orientation.X),msgs2);
orientation_y=cellfun(@(m) double(m.Orientation.Y),msgs2);
orientation_z=cellfun(@(m) double(m.Orientation.Z),msgs2);
orientation_w=cellfun(@(m) double(m.Orientation.W),msgs2);
accel_x_moving=cellfun(@(m) double(m.LinearAcceleration.X),msgs2);
accel_y_moving=cellfun(@(m) double(m.LinearAcceleration.Y),msgs2);
mag_x_moving=cellfun(@(m) double(m.MagneticField.X),msgs2);
mag_y_moving=cellfun(@(m) double(m.MagneticField.Y),msgs2);
mag_z_moving=cellfun(@(m) double(m.MagneticField.Z),msgs2);
gyro_x=cellfun(@(m) double(m.AngularVelocity.X),msgs2);
gyro_y=cellfun(@(m) double(m.AngularVelocity.Y),msgs2);
gyro_z=cellfun(@(m) double(m.AngularVelocity.Z),msgs2);
timesec=cellfun(@(m) double(m.Header.Stamp.Sec),msgs2);
timensec=cellfun(@(m) double(m.Header.Stamp.Nsec),msgs2)/1000000000.0;
totaltime=(double(timesec)+double(timensec));
initial_t=totaltime(1);
for timecounter = 1:length(totaltime)
    totaltime(timecounter)=(totaltime(timecounter)-initial_t);
end

%IMU DATA WHILE STATIONARY
accel_x_stationary=cellfun(@(m) double(m.LinearAcceleration.X),msgs4);

%IMU DATA IN CIRCLES
mag_x_circle=cellfun(@(m) double(m.MagneticField.X),msgs1);
mag_y_circle=cellfun(@(m) double(m.MagneticField.Y),msgs1);
mag_z=cellfun(@(m) double(m.MagneticField.Z),msgs1);
accel_x_circle=cellfun(@(m) double(m.LinearAcceleration.X),msgs1);
accel_y_circle=cellfun(@(m) double(m.LinearAcceleration.Y),msgs1);
accel_z=cellfun(@(m) double(m.LinearAcceleration.Z),msgs1);

% GPS DATA WHILE MOVING
utm_e=cellfun(@(m) double(m.UtmEasting.Data),msgs3);
utm_n=cellfun(@(m) double(m.UtmNorthing.Data),msgs3);
gps_time=cellfun(@(m) double(m.Header.Seq),msgs3);

% TO FIND HARD AND SOFT IRON DISTORTIONS
mag_x_circle=mag_x_circle(510:end);
mag_y_circle=mag_y_circle(510:end);
offset_x=(max(mag_x_circle)+min(mag_x_circle))/2;
offset_y=(max(mag_y_circle)+min(mag_y_circle))/2;

avg_del_x=(max(mag_x_circle)-min(mag_x_circle))/2;
avg_del_y=(max(mag_y_circle)-min(mag_y_circle))/2;

avg_del=(avg_del_x+avg_del_y)/2;
scale_x=avg_del/avg_del_x;
scale_y=avg_del/avg_del_y;

corrected_x=(mag_x_circle-offset_x)*scale_x;
corrected_y=(mag_y_circle-offset_y)*scale_y;

%PLOTTING MAGNETOMETER DATA
figure(1)
scatter(mag_x_circle,mag_y_circle,'blue')
grid on
hold on
scatter(corrected_x,corrected_y)
xlabel('Magnetic field in X (Tesla)')
ylabel('Magnetic field in Y (Tesla)')
legend('Raw data before correction','Data after correction')
title('Magnetometer Calibration')
hold off

% CORRECTING MAGNETOMETER MOVING DATA

corrected_mag_x_moving=(mag_x_moving-offset_x)*scale_x;
corrected_mag_y_moving=(mag_y_moving-offset_y)*scale_y;

% CALCULATING YAW ANGLE FROM CORRECTED MAGNETOMETER DATA
%yaw_uncorrected_magno=atan2(mag_x_moving,mag_y_moving);
%yaw_uncorrected_magno_wrap=wrapToPi(yaw_uncorrected_magno);
yaw_magno=atan2(corrected_mag_x_moving,corrected_mag_y_moving);
yaw_magno=yaw_magno-1.8383;
yaw_magno_wrap=wrapToPi(yaw_magno);

figure(2)
plot(totaltime,yaw_magno_wrap,'blue')
hold on
%min(yaw_magno);

% CALCULATING YAW FROM GYRO
yaw_gyro=cumtrapz(totaltime,gyro_z);
yaw_gyro_wrap=wrapToPi(yaw_gyro);
plot(totaltime,yaw_gyro_wrap,'red')
%min(yaw_gyro_wrap)

% CALCULATING YAW FROM SENSOR
quat=[orientation_w orientation_x orientation_y orientation_z];
eul=quat2eul(quat,'XYZ');
yaw_sensor=eul(:,3);
yaw_sensor_unwrap=unwrap(yaw_sensor);
%plot(totaltime,yaw_sensor,'magenta')
%max(yaw_sensor_unwrap)

%COMPLIMENTARY FILTER
alpha=0.985;
corrected_yaw=zeros(40147,1);
corrected_yaw(1,1)=0;
for i =1:length(yaw_sensor)-1
    corrected_yaw(i+1,1)=alpha*yaw_gyro(i,1) + (1-alpha)*yaw_magno(i,1);
    corrected_yaw(i,1)=corrected_yaw(i+1,1);
end
corrected_yaw_wrap=wrapToPi(corrected_yaw);
corrected_yaw_nobias=corrected_yaw_wrap-0.3929;
corrected_yaw_nobias=wrapToPi(corrected_yaw_nobias);


%PLOTTING FILTERED YAW WITH GYRO AND MAGNO YAW
plot(totaltime,corrected_yaw_wrap,'black');
grid on

legend('yaw-magno','yaw-gyro','yaw-filtered')
xlabel('Time(s)')
ylabel('Yaw(rad)')
title('Gyro yaw v/s Magno yaw v/s filtered yaw ')
hold off

% PLOTTING FILTERED YAW WITH YAW FROM IMU (GROUND TRUTH)
figure(3)
plot(totaltime,corrected_yaw_nobias,'blue')
grid on
hold on
plot(totaltime,yaw_sensor)
xlabel('Time(s)')
ylabel('Yaw(rad)')
title('Filtered yaw v/s IMU Yaw (ground truth)')
legend('Filtered yaw with bias removed','Ground truth')

% PLOTTING ACCELERATION IN X WITH TIME
figure(4)
%plot(totaltime,accel_x_moving)

% REMOVING BIAS 
accel_bias=mean(accel_x_stationary); %The mean acceleration in x obtained when stationary
accel_nobias=zeros(40147,1);
for p =1:length(accel_x_moving)-1
    if p<600
        accel_nobias(p,1)=accel_x_moving(p,1);
    else
        accel_nobias(p,1)=accel_x_moving(p,1)-accel_bias;   
    end
end

% plot(totaltime,accel_nobias,'blue')
% hold on

% ADJUSTING ACCELERATION
accel_adjusted=zeros(40147,1);
for i = 201:length(accel_adjusted)
    if abs(accel_nobias(i-200:i,1))<0.55 % Finding the points where car stopped
           accel_adjusted(i-200:i,1)=0;  % Setting those points to 0
           plot(totaltime(i-200:i,1),accel_nobias(i-200:i,1),'blue')
           hold on
    else
        accel_adjusted(i,1)=accel_nobias(i,1);
    end
end

plot(totaltime,accel_adjusted,'red') % Plotting filtered Acceleration with time
grid on
xlabel('Time(s)')
ylabel('Acceleration in X (m/s^2)')
title('Acceleration with time')
legend('Stoppage points','Adjusted Acceleration')
hold off

% CALCULATING FORWARD VELOCITY FROM ACCELERATION
vel_accel=cumtrapz(totaltime,accel_x_moving);

% CALCULATING FORWARD VELOCITY FROM ADJUSTED ACCELERATION
vel_adjusted=cumtrapz(totaltime,accel_adjusted);

for l=1:length(vel_adjusted)
    if vel_adjusted(l,1)<0
        vel_adjusted(l,1)=0;
    end
end

%ADJUSTING VELOCITY OBTAINED
error =0;
vel_adjusted_final=zeros(length(vel_adjusted),1);
start=1;
for k=200:200:length(vel_adjusted)
    
    if std(vel_adjusted(start:k,1))<0.0001
        error=mean(vel_adjusted(start:k,1));
        vel_adjusted_final(start:k,1)=0;
    else 
        vel_adjusted_final(start:k,1)=vel_adjusted(start:k,1)-error;
        
    end
    start=start+200;
end

for n=1:length(vel_adjusted_final)
    if vel_adjusted_final(n,1)<0
        vel_adjusted_final(n,1)=0;
    end
end

% PLOTTING VELOCTY
figure(5)
plot(totaltime,vel_accel,'magenta')
grid on
hold on

% CALCULATING VELOCITY FROM GPS
time=zeros(length(gps_time),1);
for j =1:length(gps_time)-1
    time(j+1,1)=time(j,1)+1;
end

gpsvel=zeros(length(utm_n),1);
del_utme=zeros(length(utm_n),1);
del_utmn=zeros(length(utm_n),1);
for i =2:length(utm_n)-1
    del_utme=utm_e(i+1,1)-utm_e(i,1);
    del_utmn=utm_n(i+1,1)-utm_n(i,1);
    del_dist=sqrt(del_utme^2+del_utmn^2);
    del_time=gps_time(i+1,1)-gps_time(i,1);
    velocity=del_dist/del_time;
    gpsvel(i,1)=gpsvel(i,1)+velocity;
end

plot(time,gpsvel,'red')
plot(totaltime,vel_adjusted_final,'blue')
xlabel('Time(s)');
ylabel('Velocity (m/s)');
legend('velocity from accelerometer','GPS velocity','Adjusted velocity');
title('Velocity v/s time')
hold off

figure(6)
%plot(totaltime,vel_adjusted,'blue')
plot(time,gpsvel,'red')
grid on
hold on
plot(totaltime,vel_adjusted_final,'blue')
title('Comparing adjusted velocity with ground truth')
legend('GPS velocity','Adjusted velocity from IMU')
hold off


%INTEGRATING ACCELERATION IN X TO GET X_DOT
figure(7)
X_dot=cumtrapz(totaltime,accel_x_moving);
%X_double_dot=cumtrapz(totaltime,x_dot);
accel_from_X_dot=gyro_z.*(X_dot);

plot(totaltime,accel_from_X_dot,'red');
hold on
grid on 
plot(totaltime,accel_y_moving,'blue')
xlabel('Time(s)')
ylabel('Acceleration')
legend('\omega x\prime','Observed acceleration in y (y\prime\prime)')
title('Comparing \omega x\prime and y\prime\prime','Interpreter','tex')
hold off

%PLOTTING GPS DATA
figure(8)
plot(0.000001*utm_e,0.000001*utm_n,'blue')
grid on
hold on

% ESTIMATING TRAJECTORY USING IMU
v_e=-vel_adjusted_final.*cos(corrected_yaw_nobias+1.75); %Correctng yaw to align with true data
v_n=vel_adjusted_final.*sin(-corrected_yaw_nobias+1.75);
% v_e=-vel_adjusted_final.*cos(yaw_magno_wrap+1.45); %Correctng yaw to align with true data
% v_n=vel_adjusted_final.*sin(yaw_magno_wrap+1.45);

x_e=cumtrapz(totaltime,v_e);
x_n=cumtrapz(totaltime,v_n);
x_e=1.2*(x_e)+327861.339673587; %Scaling and shifting the estimated trajectory
x_n=1.2*(x_n)+4689326.15538608;
plot(0.000001*x_e,0.000001*x_n,'red')
xlabel('UTM E (m)')
ylabel('UTM N (m)')
legend('Trajectory from GPS (ground truth)','Estimated trajectory ')
title('Estimating trajectory (with yaw from magnetometer)')
hold off
 
%ESTIMATING X_C

diff_omega_dot=diff(gyro_z);
x_c=linsolve(-diff_omega_dot,(gyro_z(1:40146).*vel_adjusted_final(1:40146)));
x_c_final=mean(x_c)/10; %(metres)
