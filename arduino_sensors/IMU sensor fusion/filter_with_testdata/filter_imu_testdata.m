clear;clc;clf;

FUSE = imufilter(SampleRate=40);
acceleration = 'a.csv';
ang_vel = 'omega.csv';

% Read the CSV file into a matrix
acc_matrix = readmatrix(acceleration);
ang_matrix = readmatrix(ang_vel);

[orientation,angularVelocity] = FUSE(acc_matrix,ang_matrix);

%{
quat_matrix = compact(orientation);
tempColumn = quat_matrix(:,1);
quat_matrix(:,1) = quat_matrix(:,4);
quat_matrix(:,4) = tempColumn;
orientation = quaternion(quat_matrix); 
%}

eulerAngles = quat2eul(orientation, "ZYX");
groundtruth = readmatrix('euler_gt.csv');
groundtruth(1278, :) = [];

time = [1:1277];
time = time';
%{
deltatime = readmatrix("dt.csv");
for s = 1:1276
    time(s+1,1) = time(s,1) + deltatime(s,1);
end 
%}

subplot(3, 1,1);
plot(time,groundtruth(:,1),'-b');
hold on;
plot(time,eulerAngles(:,1));

subplot(3, 1,2);
plot(time,groundtruth(:,2),'-b');
hold on;
plot(time,eulerAngles(:,2));

subplot(3, 1,3);
plot(time,groundtruth(:,3),'-b');
hold on;
plot(time,eulerAngles(:,3));


