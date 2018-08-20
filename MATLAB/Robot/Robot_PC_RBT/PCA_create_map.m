% -----------------------------------------------------------
% Author: AxDante <knight16729438@gmail.com>
% Singapore University of Technology and Design
% Created: August 2018
% Modified: August 2018
% -----------------------------------------------------------

% Instructions: This is a matlab file that creates a navigation map and
% save it as a ".mat" file for the main path planning algorithm. Please
% specify the waypoint patterns along with the obstacle map used here.

file_name = 'gbpp_10_01';

obsmap_name = 'obs_10_01';

grid_size = [10 16];        % Map grid size
rcg = [2 2];        % Robot starting center grid
robot_Form = 1;   % Robot starting shape

Allow = [4 6 6];    % [repeat, up, down]

% Robot waypoints
create_Wp = [1 2;
                    9 4;
                    9 6;
                    1 8;
                    1 10;
                    9 12;
                    9 14;
                    1 16];

Row_sweep_sequence = [1 4; 5 8; 9 12; 13 16];                     
                     
                     
save(['navmap/', file_name], 'file_name','obsmap_name', 'grid_size', 'rcg', 'robot_Form', ...
    'create_Wp', 'Row_sweep_sequence', 'Allow')