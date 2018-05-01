clear
clc

exp_folder = '05_180deg'; % Specify which folder is being analyzed
pathname = ['C:\Marvelmind\dashboard\logs\Experiment\', exp_folder] ;% Change the path to the folder with all the data

% Read IMU Files
if exist([pathname, '\IMU', '.txt'], 'file') == 2
    IMU = csvread([pathname,'\IMU', '.txt']);
    disp(['IMU.txt loaded successfully!']);
end

% Read UWB Files
if exist([pathname, '\TestLog', '.txt'], 'file') == 2
    TestLog = csvread([pathname,  '\TestLog', '.txt']);
    disp(['TestLog.txt loaded successfully!']);
end

H1_T = []; %Reciver 1 timestep (1 col)
H1_X = []; %Reciver 1 x (1 col)
H1_Y = []; %Reciver 1 y (1 col)
H2_T = []; %Reciver 2 timestep (1 col)
H2_X = []; %Reciver 2 x (1 col)
H2_Y = []; %Reciver 2 y (1 col)

% Reading Receiver Data 
for intidx = 1:size(TestLog,1)
    % 4th col in the datalog represent receiver ID, here we are using receivers with
    % ID = 1 and ID = 4
    % 5th col is the x value, while 6th col is the y value
    if TestLog(intidx,4) == 1  
        H1_T = [H1_T; TestLog(intidx,1)];
        H1_X = [H1_X; TestLog(intidx,5)];
        H1_Y = [H1_Y; TestLog(intidx,6)];
    elseif TestLog(intidx,4) == 4
        H2_T = [H2_T; TestLog(intidx,1)];
        H2_X = [H2_X; TestLog(intidx,5)];
        H2_Y = [H2_Y; TestLog(intidx,6)];
    end
end

% Calculate the final row size of the results, if a receiver has more
% readings than the other, remove the last few rows 
row_diff = size(H1_T,1)- size(H2_T,1);
if row_diff > 0
    H1_T(end+1-row_diff:end, :) = [];
    H1_X(end+1-row_diff:end, :) = [];
    H1_Y(end+1-row_diff:end, :) = [];
elseif row_diff<0
    H2_T(end+1+row_diff:end, :) = [];
    H2_X(end+1+row_diff:end, :) = [];
    H2_Y(end+1+row_diff:end, :) = [];
end


IMU_T = []; 
Acc = [];
Gyro= [];

% Reading IMU Data 
for intidx = 1:size(IMU,1)
    % Remove Gyro data with significant shift
    if ~(IMU(intidx, 5) < -100 || IMU(intidx, 6) < -100 || IMU(intidx, 7) < -100 ...
            || IMU(intidx, 5) > 100 || IMU(intidx, 6) > 100 || IMU(intidx, 7) > 100)
    IMU_T = [IMU_T; IMU(intidx, 1)];
    Acc = [Acc; IMU(intidx, 2) IMU(intidx, 3) IMU(intidx, 4)];
    Gyro = [Gyro; IMU(intidx, 5) IMU(intidx, 6) IMU(intidx, 7)];
    end
end

% Calculation of the angle for UWB heading
Angle = atan2((H1_Y-H2_Y),(H1_X-H2_X))*180/pi;

% Initialize the starting time to 0
H1_T = H1_T - ones(size(H1_T,1),1)*H1_T(1);
H2_T = H2_T - ones(size(H2_T,1),1)*H2_T(1);
IMU_T = IMU_T - ones(size(IMU_T,1),1)*IMU_T(1);

% Figure 1: UWB-based heading angle
figure(1)
plot(H1_T, Angle)
title('UWB-based Heading Angle')
xlabel('Time(ms)') 
ylabel('Angle(deg)') 

% Figure 2: Accelerometer
figure(2)
plot(IMU_T, Acc(:,1), IMU_T, Acc(:,2), IMU_T, Acc(:,3))
title('Accelerometer Readings')
xlabel('Time(ms)') 
ylabel('Gravity of Earth (g)') 

% Figure 3: Gyroscope
figure(3)
plot(IMU_T, Gyro(:,1), IMU_T, Gyro(:,2), IMU_T, Gyro(:,3))
title('Gyroscope Readings')
xlabel('Time(ms)') 
ylabel('Degree per Second (dps)') 