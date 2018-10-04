function d = distance(inputcities)
% DISTANCE
% d = DISTANCE(inputcities) calculates the distance between n cities as
% required in a Traveling Salesman Problem. The input argument has two rows
% and n columns, where n is the number of cities and each column represent
% the coordinate of the corresponding city. 

d = 0;
for n = 1 : length(inputcities)
    if n == length(inputcities)
        d = d + norm(inputcities(:,n) - inputcities(:,1));
    else    
        d = d + norm(inputcities(:,n) - inputcities(:,n+1));
    end
end
