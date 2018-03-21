%{
delete(instrfindall);

arduino=serial('COM6','BaudRate',9600);


fopen(arduino);
fprintf(arduino,'%s', 'H')
%out = fscanf(arduino);
fclose(arduino)
delete(arduino)
clear arduino
%}
writeDataSeries = ['2' '2' '2' 'F' 'R' 'F' 'L' 'B' 'L' 'R'];


delete(instrfindall);

arduino=serial('COM12','BaudRate',9600);
fopen(arduino);
%pause(3);
%readData = fscanf(arduino, '%s', 1) %reads "Ready" 
%writedata=uint16(500); %0x01F4

%fwrite(arduino,writedata,'uint16') %write data
%fwrite(arduino,writedata,'char')

 for i=1:8 %read 2 lines of data
    writedata= char(writeDataSeries(i));
    fwrite(arduino,writedata,'char')
    pause(2);
 end

 fclose(arduino);
 delete(arduino);