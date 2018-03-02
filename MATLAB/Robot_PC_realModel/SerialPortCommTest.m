delete(instrfindall);

arduino=serial('COM6','BaudRate',9600);
set(arduino,'DataBits',8);
set(arduino,'StopBits',1);

fopen(arduino);
for i=1:100
    fprintf(arduino,'%s', 'H')
    output = fscanf(arduino,'%s')
end