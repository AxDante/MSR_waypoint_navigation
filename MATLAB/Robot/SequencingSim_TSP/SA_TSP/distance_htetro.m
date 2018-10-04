function d = distance_htetro(hTetro_Config)
% DISTANCE
% d = DISTANCE(inputcities) calculates the distance between n cities as
% required in a Traveling Salesman Problem. The input argument has two rows
% and n columns, where n is the number of cities and each column represent
% the coordinate of the corresponding city. 

% PingCheng Edit:
% The input is now hTetro_Config
% hTetro_Config has n_blks*2 columns and n_rows rows, where n is the number of blocks
% in the robot platform, while n_rows is the number of waypoints. The columns
% now represent the x and y position of all robot blocks on the map. An
% example is as  follow (for a 4-block robot):
%
%                                    
% hTetro_Config = [ blk1.x blk2.x blk3.x blk4.x    blk1.y blk2.y blk3.y blk4.y ;
%                                blk1.x blk2.x blk3.x blk4.x    blk1.y blk2.y blk3.y blk4.y ]
%
d = 0;
%d = [0 0 0 0];
n_rows = size(hTetro_Config,1);
n_blks = (size(hTetro_Config,2)-1)/2;
for idxrow = 1 : n_rows
    %if idxrow == n_rows
     %   for idxblk =  1: n_blks
     %       d(idxblk) = d(idxblk) + norm(hTetro_Config(idxrow,[idxblk*2, idxblk*2+1]) - hTetro_Config(1,[idxblk*2, idxblk*2+1]));
     %  end
    %else    
    %    for idxblk =  1: n_blks
    %        d(idxblk) = d(idxblk) + norm(hTetro_Config(idxrow,[idxblk*2, idxblk*2+1]) - hTetro_Config(idxrow+1,[idxblk*2, idxblk*2+1]));
    %    end
    %end
    if idxrow ~=n_rows
        d = d + norm(hTetro_Config(idxrow,[2, 3]) - hTetro_Config(idxrow+1,[2, 3]));
    end
end
d = mean(d);
end
