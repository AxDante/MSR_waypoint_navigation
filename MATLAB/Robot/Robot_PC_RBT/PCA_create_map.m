% -----------------------------------------------------------
% Author: AxDante <knight16729438@gmail.com>
% Singapore University of Technology and Design
% Created: August 2018
% Modified: August 2018
% -----------------------------------------------------------

% Instructions: This is a matlab file that creates a navigation map and
% save it as a ".mat" file for the main path planning algorithm. Please
% specify the waypoint patterns along with the obstacle map used here.

file_name = 'gbpp_10_16_01'; 

obsmap_name = 'obs_10_16_01';

grid_size = [10 16];        % Map grid size (row, column)
rcg = [2 2];        % Robot starting center grid (row, column)
robot_Form = 1;   % Robot starting shape

Allow = [2 4 4 2];    % The allowed quota for [repeat, up, down, shapeshift] movements

% Robot waypoints
create_Wp = [1 2;
                    9 4;
                    9 6;
                    1 8;
                    1 10;
                    9 12;
                    9 14;
                    1 16];

Row_sweep_sequence = [1 4; 5 8; 9 12; 13 16];  % Column width                  
                     
                     
save(['navmap/', file_name], 'file_name','obsmap_name', 'grid_size', 'rcg', 'robot_Form', ...
    'create_Wp', 'Row_sweep_sequence', 'Allow')