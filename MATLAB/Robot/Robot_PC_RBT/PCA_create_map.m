% -----------------------------------------------------------
% Author: AxDante <knight16729438@gmail.com>
% Singapore University of Technology and Design
% Created: August 2018
% Modified: August 2018
% -----------------------------------------------------------

% Instructions: This is a matlab file that creates a navigation map and
% save it as a ".mat" file for the main path planning algorithm. Please
% specify the waypoint patterns along with the obstacle map used here.

filename = 'gbp_10_01';

obsmap_name = 'obs_10_01';

grid_size = [10,10];        % Map grid size
rcg = [2, 2];        % Robot starting center grid
robot_Form = 2;   % Robot starting shape

% Robot waypoints
create_Wp = [9 2 2;
                   9 3 2;
                   2 3 2;
                   2 6 2;
                   9 6 2;
                   9 8 2;
                   2 7 2;
                   2 9 2;
                   9 10 2];

% Robot sweeping rows
create_Row_sweep = [1 2;
                         0 0;
                         3 4;
                         0 0;
                         5 6;
                         0 0;
                         7 8;
                         0 0;
                         9 10];
                     
save(['navmap/', filename], 'obsmap_name', 'grid_size', 'rcg', ...
    'create_Wp', 'create_Row_sweep')