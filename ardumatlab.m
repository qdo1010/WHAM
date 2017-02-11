%%this only take in gyro and accel at the moment
%%% no heart rate yet
delete(instrfindall);
clear all;
clc;
close all;
s = serial('/dev/cu.usbmodem1411'); %%change to the right port here
set(s, 'InputBufferSize',246);
set(s, 'BaudRate', 115200);
set(s, 'DataBits',8);
fopen(s);
s.ReadAsyncMode = 'continuous';
datanum = 500; %%number of data to collect
j = 1;
while (j <= datanum) %% we can also just do while true .. 
       temp = fscanf(s,'%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n',[1 9]);
        data(j,:) = temp; %%store that in a matrix call data
        %%%%Temp is a 6 by 1 column vector
        %%temp(1) is gyrox
        %%temp(2) is gyroy
        %%temp(3) is gyroz
        %%temp(4) is accelx
        %%temp(5) is accely
        %%temp(6) is accelz
        %%temp(7) is velox
        %%temp(8) is veloy
        %%temp(9) is veloz
        %%legend('HeartRate', 'GYROX', 'GYROY', 'GYROZ', 'ACCELX', 'ACCELY', 'ACCELZ');
        %%plot(j, data(j,1),'-*',j, data(j,2),'-*', j, data(j,3), '-*',j, data(j,4),'-*',j, data(j,5), '-*',j, data(j,6),'-*', j, data(j,7), '-*');  %%plot accelx
        subplot(3,3,1), plot(j,data(j,1),'-*r');
        ylabel('deg/s'); title('X-GYRO'); 
        hold on
        subplot(3,3,2), plot(j,data(j,2),'-*b');
        ylabel('deg/s'); title('Y-GYRO');
        hold on
        subplot(3,3,3), plot(j,data(j,3),'-*g');
        ylabel('deg/s'); title('Z-GYRO');
        hold on
        subplot(3,3,4), plot(j,data(j,4),'-*r');
        title('X-ACCEL');
        hold on
        subplot(3,3,5), plot(j,data(j,5),'-*b');
        title('Y-ACCEL');
        hold on
        subplot(3,3,6), plot(j,data(j,6),'-*g');
        title('Z-ACCEL');
        hold on
        subplot(3,3,7), plot(j,data(j,7),'-*r');
        ylabel('m/s'); title('X-VELO');
        hold on
        subplot(3,3,8), plot(j,data(j,8),'-*b');
        ylabel('m/s'); title('Y-VELO');
        hold on
        subplot(3,3,9), plot(j,data(j,9),'-*g');
        ylabel('m/s'); title('Z-VELO');
        hold on
        drawnow;
        flushinput(s); %%delete the previous input to free memory
        j = j+1; %%increment 
end
stopasync(s);
fclose(s);
delete(s);