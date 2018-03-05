%{
for intidx = 1:100
    fid = fopen('C:\Marvelmind\dashboard\logs\hedgehogs.log','rt');
    A = textscan(fid, '%f %f', 'Delimiter', ',' , 'collectoutput', true, 'HeaderLines', );
    x(intidx) = A(4)
    y(intidx) = A(5)
    pause(2);
    fclose(fid);
end
%}
LastNewA = [0.1 0.1];
figure(1)
hold on
for intidx = 1:400000
    fid = fopen('C:\Marvelmind\dashboard\logs\TestLog.txt','rt');
    A = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', 'delimiter', ',','collectoutput',true...
          , 'HeaderLines', 10);
    A=A{1};
    fclose(fid);
    n=size(A,1);
    if  (NewA(1) ~= 0 &&  ~isnan(NewA(1))  &&  NewA(2) ~= 0 &&  ~isnan(NewA(2)) )
        if (LastNewA(1) ~= NewA(1) || LastNewA(2) ~= NewA(2)) 
            line([LastNewA(1) NewA(1)], [LastNewA(2) NewA(2)]);
        end
        LastNewA = NewA;
    end
    pause(0.5)
end

