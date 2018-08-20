clf
cla
close all;
clc;
clear

%% Variable Setup

% Add functions folder to the path
addpath([erase(mfilename('fullpath'),mfilename), 'functions'])

% General Map Setup
navmap_name = 'gbpp_10_01';  % Navigation map shoud be saved in '.mat' format

grid_w = 25;    % Grid width (unit:cm)
tol_wp = 4;     % Waypoint tolerance (unit:cm)               
tol_line_width =  10;   % Route deviation tolerance (unit:cm)
cvg_sample_side = [20 20];  % Robot map coverage samples size 

starting_pause_time = 1;

% Grid Map Setup
clims = [-1000, 200];  % Grid color map limits
grid_coverage_grid_default_value = -980;
grid_coverage_colormap = 'parula'; % Colormap format
grid_coverage_sim_increase = 8; % Grid color map value increased for each step during simulation;

% Algorithms
navigation_mode = 'GBPP';
wp_gen_set = 'demo01';
zigzag_mode = 'simple';

% Time Frame Setup
max_step = 1500;   % Maximum system steps
interval_system_time = 2;   % Robot dynamics update intervals
interval_normal_linear_command_send = 15; % Robot normal linear commands sending interval
interval_rotation_command_send = 10;    % Robot rotation commands sending interval

% Robot Dynamics Setup
tol_transform = pi/50;  % Robot Transformation angle tolerance (unit:rad)
Dy_angv_transform = pi/8;  % Robot transformation angular velocity (unit:rad)
tol_heading = pi/7; % Robot heading deviation tolerance (unit:rad)

update_rate_streaming = 1;  % Robot position update rate during data streaming
update_rate_streaming_single_value = 1; % Robot position update rate during data streaming with single values
update_rate_sim = 0.2; % Robot position update rate during simulation


% Toggle 
is_calculate_coverage = false;
is_calculate_grid_coverage_duration = true;
is_display_coverage_map = false;
is_display_wp = true;   
is_display_wp_clearing = false;
is_display_route_clearing = true;
is_display_grid_on = true;
is_display_obstacle = true;
is_display_grid_coverage_map = false;
is_display_ignite_swept_grid = true;
is_display_rbt_center = true;
is_print_coverage = false;
is_print_sent_commands = true;
is_print_route_deviation = false;
is_fixed_offset = false;
is_sim_normal_noise_on= false;
is_sim_large_noise_y_on = false;
is_sim_heading_correction = false;
is_save_path_planning_data = true;


% Simulation Setup
sim_noise_normal = 20;  % Noise value during simulation (unit: cm)
sim_noise_large_y_frequency = 0.05; % Frequency of large Y axis noise during simulation 
sim_noise_large_y = 50; % large Y axis noise value during simulation (unit: cm)


%% Navigation / Obstacle Map Loading

Map_obs = []; % Obstacle map array initialization

% Navigation map check
if strcmp(navmap_name,'')
    disp(['No navigation map detected! Please sepecify the navigation map!'])
    return
end
navmap_path = ['navmap/',navmap_name, '.mat'];
if exist(navmap_path, 'file') ~= 2
    disp(['Navigation file ', navmap_path, ' unfound! Aborting...'])
    return
end

% Loading navigation map
disp(['Loading navigation map: ', navmap_path])
load(navmap_path)

% Obstacle map check
if strcmp(obsmap_name,'')
    disp('No obstacle map detected! Aborting...')
    return
end
obsmap_path = ['obsmap/',obsmap_name, '.dat'];
if exist(obsmap_path, 'file') ~= 2
    disp(['Obstacle file ', obsmap_path, ' unfound! Aborting...'])
    return
end

% Loading the obstacle map
disp(['Loading obstacle map: ', obsmap_path])
Map_obs = csvread(obsmap_path);
disp('Successfully loaded navigation and obstacle map.')
disp('=========================')

%% Waypoint Generation

if (strcmp(navigation_mode,'GBPP'))
    
    if(is_save_path_planning_data)
        file_number = 0;
        while (file_number <= 99)
            datalog_path = ['datalog/', file_name, '_', num2str(file_number,'%02.f'),'.csv'];
            if exist(datalog_path, 'file') ~= 2
                fid = fopen(datalog_path, 'w') ;
                break;
            end
            file_number = file_number + 1;
        end
    end    
    
    disp('Navigation mode set to Graph-theory Based Path Planning')
    disp('Generating waypoints...')
    wp_current = 1;
    if (strcmp(wp_gen_set,'demo01'))
        [Wp, Wp_hack] = PCA_generate_waypoint(grid_size, grid_w, Map_obs, rcg, create_Wp, Row_sweep_sequence, robot_Form, Allow);
    end
    pos_uwb_offset = (rcg-0.5)*grid_w;
    fig_1_title_name = 'GBPP Waypoint Map';
    
    if(is_save_path_planning_data)
        fclose(fid);
    end   
    
else
    disp('Navigation method is invalid.')
    disp('Terminating Matlab script...')
    return
end

%% Variable initialization

time_pause = interval_system_time/700;  % Time pause between each robot action
pos_uwb_raw =  zeros(2, max_step);
pos_uwb = zeros(2, max_step);
pos_center = zeros(4, 2, max_step);

Grid_setup = zeros(grid_size(2),  grid_size(1));
Grid_current =  zeros(grid_size(2),  grid_size(1), max_step);
Grid_visited =  ones(grid_size(2),  grid_size(1), max_step)* grid_coverage_grid_default_value;
Grid_obstacle = zeros(grid_size(2),  grid_size(1), max_step);

for idxobs = 2:size(Map_obs,1)
    Grid_visited(Map_obs(idxobs,1),Map_obs(idxobs,2),1) = clims(1);
    Grid_obstacle(Map_obs(idxobs,1),Map_obs(idxobs,2),1) = 1;
end

robot_center_Grid = [];
robot_Grid = [];

prev_char_command = 'S';
command_count_normal_linear = 0;
command_count_rotation = 0;

Dy_force = zeros(4, 2, max_step);       % Simulated movement force
Dy_a = zeros(4, 2, max_step);            % Simulated movement acceleration
Dy_v = zeros(4, 2, max_step);            % Simulated movement velocity

is_rotating = false;


Obstacles = [];

Line_Linear_Route = [];

Line_Robot = [];
Line_Robot_area = [];
Line_Border = [];

is_transforming = false;

initRobotShapes = [0 0 0 0 ;              
                        0 0 pi pi;
                        -pi 0 pi/2 -pi/2;
                        -pi 0 0 0;
                        -pi 0 0 -pi;
                        -pi/2 0 pi 0;
                        -pi 0 pi/2 pi/2];
                    
RobotShapes = [initRobotShapes; initRobotShapes + pi/2*1];
RobotShapes = [RobotShapes; initRobotShapes + pi/2*2];
RobotShapes = [RobotShapes; initRobotShapes - pi/2*1];
                    
%                                    Robot Shapes
%   =====================================
%         s01     s02      s03     s04      s05        s06       s07
%   -----------------------------------------------------------------
%
%          4                  3 4       4
%          3       2 3       2         3       4 3        2 3 4     2 3
%          2       1 4       1       1 2         2 1        1           1 4
%          1                      
%
%         s08           s010         s11        s12       s13      s14
%   -----------------------------------------------------------------
%
%                        1 2 4        1              4         2            2
%       1 2 3 4            3         2 3 4      2 3      1 3         1 3
%                                                     1           4         4
%  

Robot_Relative_Pos = [0 -1; 0 1; 0 2;
                                 0 -1; 1 0; 1 -1;
                                 -1 0; 0 1; 1 1;
                                 -1 0; 0 1; 0 2;
                                 1 0; 0 1; -1 1;
                                 1 -1; 1 0; 2 0;
                                 1 -1; 1 0; 2 -1];
                             
heading = [0 0 0 0];
for intidx = 1:4
    heading(intidx)= RobotShapes(robot_Form,intidx);
end                

Char_command_array_linear = ['R', 'F', 'L', 'B'];
Char_command_array_linear_adjustment =  ['r', 'f', 'l', 'b'];
heading_command_compensate = 0;
 
char_command = 'S';
Cvg = [];
count_cvg_point = 0;
cvg_sample_w = grid_w*[grid_size(2)/cvg_sample_side(1) grid_size(1)/cvg_sample_side(2)];
grid_dhw = sqrt(2) / 2 * grid_w;

count_pos_initialize = 0;
is_pos_initialized = false;
pos_initial = [];

fig_1_title_name = '';

for idxx = 1:(cvg_sample_side(2)+1)
    for idxy = 1:(cvg_sample_side(1)+1)
        Cvg = [Cvg; cvg_sample_w(1)*(idxy-1) cvg_sample_w(2)*(idxx-1) 0];
    end
end


%% DRAW MAP

% First figure: Navigation map
figure(1)
axis([-grid_w grid_w*(grid_size(1)+1) -grid_w grid_w*(grid_size(2)+1)])
set(gcf,'units','points','position',[10,10,grid_size(1)*55,grid_size(2)*50])
title(fig_1_title_name)
hold on

% Draw waypoints 
 Circle_Wp = [];
if (is_display_wp)
    for idx = 1: size(Wp_hack,1)
        Circle_Wp(idx) = plot(Wp_hack(idx, 1), Wp_hack(idx, 2),'Color', 'r', 'LineWidth', 2, 'Marker', 'o');
    end
end

% Draw obstacles
if (is_display_obstacle)
    figure(1)
    for idxobs = 1:size(Map_obs,1)
        Obstacles = rectangle('Position', [grid_w*(Map_obs(idxobs, :) - [1 1]), grid_w grid_w], ...
                                                'FaceColor', [0 0 0]);
    end
end

txt_endLine = [0 0];
txt_endLine_last = [0 0];

%% Main Algorithm  (SW)

tic  % Start timer

if (true)
    
    % Algorithm Setup
    pos_uwb_raw(:, 1) = pos_uwb_offset;
    pos_uwb(:, 1) = pos_uwb_raw(:, 1);

    % Algorithm Main Loop
    for step = 1:max_step
        
        % Pause function
        pause(time_pause);
           
        % Grid Info
        robot_center_Grid = [1+floor(pos_uwb(1, step)/grid_w) 1+floor(pos_uwb(2, step)/grid_w)];
        rotated_relative_grid_pos = PCA_rotation_matrix(Robot_Relative_Pos, Wp(wp_current, 3));
        robot_Grid = [ rotated_relative_grid_pos(1,:);
                             0 0;
                            rotated_relative_grid_pos(2,:);
                            rotated_relative_grid_pos(3,:)] + robot_center_Grid;
                        
        Grid_visited(:,:,step+1) = Grid_visited(:,:,step);
       
        for idxbox = 1:4
            if (robot_Grid(idxbox,1) > 0 && robot_Grid(idxbox,2) > 0 &&...
                    robot_Grid(idxbox,1) <= grid_size(1) && robot_Grid(idxbox,2) <= grid_size(2))
                if (Grid_visited(robot_Grid(idxbox,1),robot_Grid(idxbox,2),step+1) == grid_coverage_grid_default_value)
                    Grid_visited(robot_Grid(idxbox,1),robot_Grid(idxbox,2),step+1) = 0;
                else
                    Grid_visited(robot_Grid(idxbox,1),robot_Grid(idxbox,2),step+1) = ...
                        Grid_visited(robot_Grid(idxbox,1),robot_Grid(idxbox,2),step) + grid_coverage_sim_increase;
                end
            end
        end
        
        % Grid coverage map
        if (is_display_grid_coverage_map)
            figure(2)
            title('hTetro Coverage Map')
            imagesc(flipud(transpose(Grid_visited(:,:,step+1))), clims)
            cmap = colormap(grid_coverage_colormap);
            cmap(1,:) = zeros(1,3);
            colormap(cmap);
            colorbar
            figure(1)
        end
        
        % Robot Transformation
        is_require_transform = false;
        for rbtidx = 1:4
            if abs(heading(rbtidx) - RobotShapes(Wp(wp_current, 3),rbtidx)) > tol_transform
                is_require_transform = true;
            end
        end
        
        if (robot_Form ~= Wp(wp_current, 3) && is_require_transform)
            if (command_count_rotation >= interval_rotation_command_send)
                is_transforming = true;
                prev_char_command = char(num2str(Wp(wp_current, 3)));
                if (is_print_sent_commands) 
                    disp(['Time:', num2str(round(toc,2)), 's; Command Sent: ''' , num2str(Wp(wp_current, 3)), ''''])
                end
                command_count_rotation = 0;
            else
                command_count_rotation = command_count_rotation + 1;
            end
        else
            robot_Form = Wp(wp_current, 3);
            heading_command_compensate = floor((Wp(wp_current, 3)-1)/7);
            is_transforming = false;
        end
            
        % Robot Motion
        if (is_transforming)
            heading = robotTransformation(Wp(wp_current, 3), heading, RobotShapes, tol_transform, Dy_angv_transform);
            pos_uwb(:, step+1) = pos_uwb(:, step);
        elseif (heading(2) > tol_heading && is_sim_heading_correction ) 
            [Dy_v(:, :, step), heading] = robotMovement('l', heading, 0);
            pos_uwb(:, step+1) = pos_uwb(:, step);
        elseif (heading(2) < - tol_heading && is_sim_heading_correction) 
            [Dy_v(:, :, step), heading]  = robotMovement('r', heading, 0);
            pos_uwb(:, step+1) = pos_uwb(:, step);
        else
            % Waypoint navigation
            if (strcmp(navigation_mode,'GBPP'))
                if(~is_require_transform && norm(pos_uwb(:, step).' - Wp(wp_current, 1:2)) > tol_wp )
                    if abs(Wp(wp_current, 1) - pos_uwb(1,step)) > abs(Wp(wp_current, 2) - pos_uwb(2,step)) 
                        if Wp(wp_current, 1) - pos_uwb(1,step) > 0
                            char_command = Char_command_array_linear(1+mod(heading_command_compensate,4));  
                        else
                            char_command = Char_command_array_linear(1+mod(heading_command_compensate+2,4));
                        end
                    else
                        if Wp(wp_current, 2) - pos_uwb(2,step) > 0
                            char_command = Char_command_array_linear(1+mod(heading_command_compensate+1,4));
                        else
                            char_command = Char_command_array_linear(1+mod(heading_command_compensate+3,4));
                        end
                    end
                else
                    char_command = 'S';
                end
            end
            % Simulation movements
            if (~is_transforming)
                [Dy_v(:, :, step), heading]  = robotMovement(char_command, heading, 2);
                sim_noise = 0;
                if (is_sim_normal_noise_on)
                    sim_noise = sim_noise + rand(2,1) * sim_noise_normal - sim_noise_normal/2.0;
                end
                if (is_sim_large_noise_y_on && rand < sim_noise_large_y_frequency) 
                    sim_noise = sim_noise + [0.5; rand] * sim_noise_large_y - sim_noise_large_y/2.0;
                end
                pos_uwb(:, step+1) = Dy_v(2, :, step).' * interval_system_time+ ...
                                                    update_rate_sim* (pos_uwb(:, step) + sim_noise)...
                                                   + (1 - update_rate_sim)* pos_uwb(:, step);
                                               
                               
            end
        end
        
        % Check if robot reaches the next waypoint
        if(norm(pos_uwb(:, step+1).' - Wp(wp_current, 1:2)) < tol_wp )
            if (is_display_wp && is_display_wp_clearing)
                delete(Circle_Wp(wp_current));
            end
            wp_current = wp_current + 1;
            char_command = 'S';
            % Break condition
            if wp_current > size(Wp,1)
                break;
            end
        end
        
        
        % calibrate pos here
        pos_x = pos_uwb(1,step);
        pos_nx = pos_uwb(1, step+1);
        pos_y = pos_uwb(2,step);
        pos_ny = pos_uwb(2, step+1);
        
        
        % Determine Robot center
        pos_center(2,:, step) = [pos_x pos_y];
        pos_center(1,:, step) =  pos_center(2,:,step) + grid_dhw* ...
                                   [sin(pi/4 - heading(2))-cos(pi/4 - heading(1)) ...
                                    -cos(pi/4 - heading(2))-sin(pi/4 - heading(1))];
        pos_center(3,:, step) =  pos_center(2,:,step) + grid_dhw* ...
                                   [cos(pi/4 - heading(2))+sin(heading(3) - pi/4) ...
                                    sin(pi/4 - heading(2))+cos(heading(3) - pi/4)];
        pos_center(4,:, step) =  pos_center(3,:,step) + grid_dhw* ...
                                   [sin(-pi/4 + heading(3))+sin(pi/4 + heading(4)) ...
                                    cos(-pi/4 + heading(3))+cos(pi/4 + heading(4))]; 
        
        pos_center(2,:, step+1) = [pos_nx pos_ny];
        pos_center(1,:, step+1) =  pos_center(2,:, step+1) + grid_dhw* ...
                                   [sin(pi/4 - heading(2))-cos(pi/4 - heading(1)) ...
                                    -cos(pi/4 - heading(2))-sin(pi/4 - heading(1))];
        pos_center(3,:, step+1) =  pos_center(2,:, step+1) + grid_dhw* ...
                                   [cos(pi/4 - heading(2))+sin(heading(3) - pi/4) ...
                                    sin(pi/4 - heading(2))+cos(heading(3) - pi/4)];
        pos_center(4,:, step+1) =  pos_center(3,:,step+1)  + grid_dhw* ...
                                   [sin(-pi/4 + heading(3))+sin(pi/4 + heading(4)) ...
                                    cos(-pi/4 + heading(3))+cos(pi/4 + heading(4))]; 
        
        
        % Robot coverage percentage calculation
        if(is_calculate_coverage)
                for cvg_idx = 1: numel(Cvg(:, 1))
                    if (abs(pos_center(2, :, step)-Cvg(cvg_idx, 1:2)) < 2.5*sqrt(2)*grid_w)
                        for robidx = 1:4
                            if (Cvg(cvg_idx, 3) == 0)
                                tri1_x = [pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 - heading(robidx)) ...
                                              pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 - heading(robidx)) ...
                                              Cvg(cvg_idx, 1)];
                                tri1_y = [pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx))  ...
                                              pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                              Cvg(cvg_idx, 2)];
                                area1 = polyarea(tri1_x,tri1_y);
                                tri2_x = [pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 -  heading(robidx)) ...
                                              pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 -  heading(robidx)) ...
                                              Cvg(cvg_idx, 1)];
                                tri2_y = [pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx))  ...
                                              pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx)) ...
                                              Cvg(cvg_idx, 2)];
                                area2 = polyarea(tri1_x,tri1_y);
                                tri3_x = [pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                              pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 - heading(robidx)) ...
                                              Cvg(cvg_idx, 1)];
                                tri3_y = [pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx))  ...
                                              pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx)) ...
                                              Cvg(cvg_idx, 2)];
                                area3 = polyarea(tri3_x,tri3_y);
                                tri4_x = [pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                              pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 -  heading(robidx)) ...
                                              Cvg(cvg_idx, 1)];
                                tri4_y = [pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx)) ...
                                              pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                              Cvg(cvg_idx, 2)];
                                area4 =  polyarea(tri4_x,tri4_y);
                                if area1 + area2 + area3 + area4 <= grid_w* grid_w + 0.1
                                    Cvg(cvg_idx,3) = 1;
                                    count_cvg_point = count_cvg_point+1;
                                end
                            end
                        end
                    end
                end
                if (is_print_coverage)
                    disp(['Coverage: ',  num2str(count_cvg_point*100 / numel(Cvg(:, 1))), ' %']);
                end
        end
                                
                               
%% Figure!!
        
        % remove previous robot line plot
        if (~isempty(Line_Robot))
            delete(Line_Robot)
            delete(Line_Linear_Route)
        end
        
        Line_Robot = [];  % Robot outer border
        Line_Border = [];  % Workspace outer border
        Line_Linear_Route = [];
        Line_RbtCent = [];  % Robot center trajectory
        
        % plot robot BG
            if (is_display_coverage_map == true)
                for robidx = 1:4
                Line_Robot(robidx,1) = line([pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 - heading(robidx)) ...
                                                         pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 - heading(robidx))], ...
                                                        [pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx)) ...
                                                         pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx))], 'Color', [255 255 204]/255, 'LineWidth', 5);
                Line_Robot(robidx,2) = line([pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 -  heading(robidx))... 
                                                         pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 -  heading(robidx))], ...
                                                        [ pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx))...
                                                          pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx))], 'Color', [255 255 204]/255, 'LineWidth', 5);
                Line_Robot(robidx,3) = line([pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                                         pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 -  heading(robidx))], ...
                                                        [ pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx)) ...
                                                          pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx))], 'Color', [255 255 204]/255, 'LineWidth', 5);
                Line_Robot(robidx,4) = line([pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                                         pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 -  heading(robidx))], ...
                                                        [ pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx))...
                                                          pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx))], 'Color', [255 255 204]/255, 'LineWidth', 5);
                 end
            end
            
            % Draw Waypoints
            if (~isempty(Circle_Wp))
                delete(Circle_Wp);
            end
            Circle_Wp = [];
            if (is_display_wp)
                for idx = wp_current: size(Wp,1)
                    Circle_Wp(idx) = plot(Wp(idx, 1), Wp(idx, 2),'Color', 'r', 'LineWidth', 2, 'Marker', 'o');
                end
            end
            
            % Draw Outer Border
            Line_Border(1) = line([0 0], [0 grid_w*grid_size(2)], 'Color', 'black', 'LineWidth', 2);
            Line_Border(2) =line([0 grid_w*grid_size(1)], [0 0], 'Color', 'black', 'LineWidth', 2);
            Line_Border(3) =line([grid_w*grid_size(1) 0], [grid_w*grid_size(2) grid_w*grid_size(2)], 'Color', 'black', 'LineWidth', 2);
            Line_Border(4) =line([grid_w*grid_size(1) grid_w*grid_size(1)], [grid_w*grid_size(2) 0], 'Color', 'black', 'LineWidth', 2);


            if (is_display_grid_on)
                for idxx = 1:(grid_size(1) + 1)
                    line(grid_w*[(idxx-1) (idxx-1)], grid_w*[0 grid_size(2)], 'Color', 'black', 'LineWidth', 0.5);
                end
                for idxy = 1:(grid_size(2) + 1)
                    line(grid_w*[0 grid_size(1)], grid_w*[(idxy-1) (idxy-1)], 'Color', 'black', 'LineWidth', 0.5);
                end
            end

            % Draw Robot Tracks
            % TODO: track rotation!
            if (strcmp(navigation_mode,'Line') && is_display_route)
                if (is_display_route_clearing)
                    wpidx_begin = wp_current;
                else 
                    wpidx_begin = 2;
                end
                for wpidx = wpidx_begin : size(Wp,1)
                    if (Wp(wpidx, 4) ~= 1)
                        route_norm_vector = null(Wp(wpidx, 1:2) - Wp(wpidx-1, 1:2)).';
                        line_linear_route_v1 = Wp(wpidx-1, 1:2) - route_norm_vector()*tol_line_width;
                        line_linear_route_v2 = Wp(wpidx, 1:2) + route_norm_vector()*tol_line_width;
                        line_linear_route_x = [line_linear_route_v1(1) line_linear_route_v2(1) line_linear_route_v2(1) line_linear_route_v1(1) line_linear_route_v1(1)];
                        line_linear_route_y = [line_linear_route_v1(2) line_linear_route_v1(2) line_linear_route_v2(2) line_linear_route_v2(2) line_linear_route_v1(2)];
                        %{route_norm_vector = null(Wp(Wpidx, 1:2) - Wp(Wpidx, 1:2)).';
                        %Line_Linear_Route = [ Line_Linear_Route, plot(line_linear_route_x, line_linear_route_y, 'b-')];
                    end
                end
            end

            % Draw Robot Outline
            for robidx = 1:4
                %line([pos_center(robidx,1,step) pos_center(robidx,1,step+1)], [pos_center(robidx,2,step) pos_center(robidx,2,step+1)])  
                Line_Robot(robidx,1) = line([pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 - heading(robidx)) ...
                                                         pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 - heading(robidx))], ...
                                                        [pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx)) ...
                                                         pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx))], 'Color', [77, 77, 255]/255, 'LineWidth', 3);
                Line_Robot(robidx,2) = line([pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 -  heading(robidx))... 
                                                         pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 -  heading(robidx))], ...
                                                        [ pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx))...
                                                          pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx))], 'Color',  [77, 77, 255]/255, 'LineWidth', 3);
                Line_Robot(robidx,3) = line([pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                                         pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 -  heading(robidx))], ...
                                                        [ pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx)) ...
                                                          pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx))], 'Color',  [77, 77, 255]/255, 'LineWidth', 3);
                Line_Robot(robidx,4) = line([pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                                         pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 -  heading(robidx))], ...
                                                        [ pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx))...
                                                          pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx))], 'Color', [77, 77, 255]/255, 'LineWidth', 3);   
            end

            
            % Draw Robot Center
            Robot_center(step,1,1:2) = [pos_x pos_nx];
            Robot_center(step,2,1:2) = [pos_y pos_ny];
            
            if (is_display_rbt_center)
                if (~isempty(Line_RbtCent))
                    delete(Line_RbtCent);
                end
                Line_RbtCent = [];
                for rbtcenidx = 1:size(Robot_center,1)
                    Line_RbtCent = [Line_RbtCent, line([Robot_center(rbtcenidx,1,1) Robot_center(rbtcenidx,1,2)], ...
                                                                    [Robot_center(rbtcenidx,2,1) Robot_center(rbtcenidx,2,2)])];
                end
            end
            
            % Time pause at the beginning (for video recording)
            if (step == 1)
                pause(starting_pause_time)
            end
        
    end
end

disp('===================');
disp('Robot Navigation Completed!');
toc
hold off
if (is_calculate_coverage)
    disp(['Final Map Coverage: ',  num2str(count_cvg_point*100 / numel(Cvg(:, 1))), ' %']);
end