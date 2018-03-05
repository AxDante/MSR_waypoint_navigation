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
writeDataSeries = ['2' 'F' 'R' '1' 'L' 'F' '2' 'R'];


delete(instrfindall);

arduino=serial('COM6','BaudRate',9600);
fopen(arduino);

readData = fscanf(arduino) %reads "Ready" 
%writedata=uint16(500); %0x01F4

%fwrite(arduino,writedata,'uint16') %write data

writedata= char('H');
fwrite(arduino,writedata,'char')

 for i=1:8 %read 2 lines of data
    fwrite(arduino,writedata,'char')
    pause(10);
    %readData = fscanf(arduino)
end
 fclose(arduino);
 delete(arduino);