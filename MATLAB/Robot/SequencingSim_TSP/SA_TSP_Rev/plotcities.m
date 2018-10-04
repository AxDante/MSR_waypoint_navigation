function f = plotcities(inputcities)
% PLOTCITIES
% PLOTCITIES(inputcities) plots the location of cities from the argument
% inputcities. Inputcities argument has 2 rows and n columns, where n is
% the number of cities. Apart from the location the symmetric route is also
% plotted.
shg
temp_1  = plot(inputcities(1,:),inputcities(2,:),'b*');
set(temp_1,'erasemode','none');
temp_2 = line(inputcities(1,:),inputcities(2,:),'Marker','*');
set(temp_2,'color','r');
x = [inputcities(1,1) inputcities(1,length(inputcities))];
y = [inputcities(2,1) inputcities(2,length(inputcities))];
xl = 10*round(min(inputcities(1,:))/10);
yl = 10*round(min(inputcities(2,:))/10);
if xl > 0
    xl = 0;
end
if yl > 0
    yl = 0;
end
xu = 10*round(max(inputcities(1,:))/10);
yu = 10*round(max(inputcities(2,:))/10);
if xu == 0
    xu = 1;
end
if yu == 0
    yu = 1;
end
axis([xl xu yl yu]) ;
temp_3 = line(x,y);
set(temp_3,'color','r');
dist = distance(inputcities);
distance_print = sprintf(...
     'The roundtrip length for %d cities is % 4.6f units'...
     ,length(inputcities),dist);
text(xl/1.05,1.05*yu,distance_print,'fontweight','bold');
drawnow;