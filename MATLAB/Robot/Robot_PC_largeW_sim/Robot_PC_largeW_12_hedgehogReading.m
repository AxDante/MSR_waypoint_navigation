clf
cla
close all;
clc;

%% Variable Setup

addpath('C:\Users\IceFox\Desktop\ERMINE\MATLAB')
addpath('C:\Users\IceFox\Desktop\ERMINE\MATLAB\Robot_PC')
addpath('C:\Users\IceFox\Desktop\ERMINE\MATLAB\Robot_PC\Maps')

% Map Setup
grid_size = [6 10];
grid_w = 25;

sendTime = 0;
stopAwhile = false;
justTurnOneDir = false;
justTurnOneDirCount = 0;
tol_wp = 5;                    
tol_line_width = 12;

cvg_sample_side = [20 20];
fixed_offset = [96.5 -54.5];
starting_grid = [1 2];

% Algorithms
Algorithm = 'square_waypoint';
navigation_mode = 'Point04';

% Time Frame Setup
max_step = 20000;
interval_system_time = 2;
interval_normal_linear_command_send = 30;
interval_rotation_command_send = 15;

% Robot Dynamics Setup
tol_transform = pi/50;
Dy_angv_transform = pi/20;
tol_heading = pi/7;

update_rate_streaming = 1;                      
update_rate_streaming_single_value = 1; 
update_rate_sim = 0.2;

% Serial Communication Setup
Serial_port = 'COM3';
baudrate = 9600;


% Toggle 
is_xbee_on = false;
is_streaming_on = false;

is_calculate_coverage = false;
is_display_coverage_map = false;
is_display_wp = true;
is_display_wp_clearing = true;
is_display_route = true;
is_display_route_clearing = true;
is_display_grid_on = true;
is_print_coverage = false;
is_print_sent_commands = true;
is_print_route_deviation = false;
is_fixed_offset = false;
is_sim_normal_noise_on= true;
is_sim_large_noise_y_on = true;
is_sim_heading_correction = false;
is_streaming_collect_single_values = false;

% Data streaming setup
streaming_max_single_val = 1000;                 % unit: cm
streaming_max_shift_dis = 45;                       % unit: cm 
streaming_max_shift_dis_single_value = 30;    % unit: cm

max_pos_initialize = 10;

% Simulation Setup
sim_noise_normal = 12;                        %unit: cm
sim_noise_large_y_frequency = 0;
sim_noise_large_y = 50;

robot_weight = [1.5 1.5 1.5 1.5];
robot_Form = 2;

%% Variable initialization


heading = [0 0 pi pi];

time_pause = interval_system_time/100;

pos_uwb_offset = [12.5 37.5];
flex_offset = [0 0];
pos_uwb_raw =  zeros(2, max_step);
pos_uwb = zeros(2, max_step);
pos_uwb_1 =  zeros(2, max_step);

pos_center = zeros(4, 2, max_step);

%heading = zeros(4, max_step);

Grid_setup = zeros(grid_size(1),  grid_size(2));
Grid_current =  zeros(grid_size(1),  grid_size(2), max_step);
Grid_visited =  zeros(grid_size(1),  grid_size(2), max_step);
robot_Grid = [];

prev_char_command = 'S';
command_count_normal_linear = 0;
command_count_rotation = 0;

Dy_force = zeros(4, 2, max_step);
Dy_a = zeros(4, 2, max_step);
Dy_v = zeros(4, 2, max_step);

is_rotating = false;

Wp = [];

Circle_Wp = [];

Line_Linear_Route = [];

Line_Robot = [];
Line_Robot_area = [];
Line_Border = [];
loc_center = [0 0];

is_transforming = false;

RobotShapes = [0 0 0 0 ;              
                        0 0 pi pi;
                        0 0 0 -pi;
                        -pi 0 0 0;
                        -pi 0 0 -pi;
                        -pi/2 0 pi 0;
                        -pi/2 0 pi pi/2];
       
for idx = 1:3
    RobotShapes = [RobotShapes; RobotShapes + pi/2*idx];
end
                    
%                                    Robot Shapes
%   =====================================
%         s01     s02      s03     s04      s05        s06       s07
%   -----------------------------------------------------------------
%
%          4                  4 3       4
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

Char_command_array_linear = ['R', 'F', 'L', 'B'];
Char_command_array_linear_adjustment =  ['r', 'f', 'l', 'b'];
heading_command_compensate = 0;

char_command = '';
Cvg = [];
count_cvg_point = 0;
cvg_sample_w = grid_w*[grid_size(1)/cvg_sample_side(1) grid_size(2)/cvg_sample_side(2)];
grid_dhw = sqrt(2) / 2 * grid_w;

count_pos_initialize = 0;
is_pos_initialized = false;
pos_initial = [];
valid_angle = 0;
heading_angle = 0;
turncheck = false;
start_sending = false;

if (is_xbee_on)
    delete(instrfindall);
    arduino = serial(Serial_port,'BaudRate',baudrate);
    fopen(arduino);
    
    % TO CHANGE!
    writedata = char('2');
    fwrite(arduino,writedata,'char');
end

for idxx = 1:(cvg_sample_side(1)+1)
    for idxy = 1:(cvg_sample_side(2)+1)
        Cvg = [Cvg; cvg_sample_w(1)*(idxx-1) cvg_sample_w(2)*(idxy-1) 0];
    end
end

%% Waypoint Generation

if (strcmp(navigation_mode,'Line'))
    wp_current = 1;
    disp('Generating robot routes...')
    for idx = 1: grid_size(1)-2
        if (mod(idx, 4) == 1)
            if (idx == 1)
                Wp = [Wp; 0.5*grid_w  (idx+0.5)*grid_w 2 1];
            else
                Wp = [Wp; 0.5*grid_w  (idx+floor(idx/2)+0.5)*grid_w 2 0];
            end
        end
        if (mod(idx, 4) == 2)
            Wp = [Wp; (grid_size(2) - 1.5)*grid_w  (idx+floor(idx/2)-1.5)*grid_w 2 0];
        end
        if (mod(idx, 4) == 3)
            if (idx == 7)
                Wp = [Wp; (grid_size(2) - 1.5)*grid_w (idx+floor(idx/2)-0.5)*grid_w 2 0];
            else
                Wp = [Wp; (grid_size(2) - 1.5)*grid_w (idx+floor(idx/2)-0.5)*grid_w 1 0];
            end
        end
        if (mod(idx, 4) == 0)
            if (idx == 8)
                Wp = [Wp; 0.5*grid_w  (idx+floor(idx/2)-2.5)*grid_w 2 0];
            else
                 Wp = [Wp; 0.5*grid_w  (idx+floor(idx/2)-2.5)*grid_w 1 0];
            end
        end
    end
elseif (strcmp(navigation_mode,'Point'))
    disp('Generating waypoints...')
    wp_current = 1;
    for idx = 1: grid_size(1)-2
         if (mod(idx, 4) == 1)
            if (idx == 1)
                Wp = [Wp; 0.5*grid_w  (idx+0.5)*grid_w 2 1];
                Wp = [Wp; (0.5+(grid_size(2)-2)*1/4.0)*grid_w  (idx+0.5)*grid_w 2 0];
                Wp = [Wp; (0.5+(grid_size(2)-2)*2/4.0)*grid_w  (idx+0.5)*grid_w 2 0];
                Wp = [Wp; (0.5+(grid_size(2)-2)*3/4.0)*grid_w  (idx+0.5)*grid_w 2 0];
            else
                Wp = [Wp; 0.5*grid_w  (idx+floor(idx/2)+0.5)*grid_w 2 0];
     
                Wp = [Wp; (0.5+(grid_size(2)-2)*1/4.0)*grid_w  (idx+2.5)*grid_w 2 0];
                Wp = [Wp; (0.5+(grid_size(2)-2)*2/4.0)*grid_w  (idx+2.5)*grid_w 2 0];
                Wp = [Wp; (0.5+(grid_size(2)-2)*3/4.0)*grid_w  (idx+2.5)*grid_w 2 0];
            end
        end
        if (mod(idx, 4) == 2)
            Wp = [Wp; (grid_size(2) - 1.5)*grid_w  (idx+floor(idx/2)-1.5)*grid_w 2 0];
        end
        if (mod(idx, 4) == 3)
            if (idx == 7)
                
                Wp = [Wp; (grid_size(2) - 1.5)*grid_w (idx+floor(idx/2)-0.5)*grid_w 2 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*1/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 2 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*2/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 2 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*3/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 2 0];
            else
                Wp = [Wp; (grid_size(2) - 1.5)*grid_w (idx+floor(idx/2)-0.5)*grid_w 1 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*1/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 1 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*2/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 1 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*3/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 1 0];
            end
        end
        if (mod(idx, 4) == 0)
            if (idx == 8)
                Wp = [Wp; 0.5*grid_w  (idx+floor(idx/2)-2.5)*grid_w 2 0];
            else
                 Wp = [Wp; 0.5*grid_w  (idx+floor(idx/2)-2.5)*grid_w 1 0];
            end
        end
    end
elseif (strcmp(navigation_mode,'Point02'))
    disp('Generating waypoints...')
    wp_current = 1;
    for idx = 1: grid_size(1)-2
         if (mod(idx, 4) == 1)
            if (idx == 1)
                Wp = [Wp; 0.5*grid_w  (idx+0.5)*grid_w 2 1];
                Wp = [Wp; (0.5+(grid_size(2)-2)*1/4.0)*grid_w  (idx+0.5)*grid_w 2 0];
                Wp = [Wp; (0.5+(grid_size(2)-2)*2/4.0)*grid_w  (idx+0.5)*grid_w 2 0];
                Wp = [Wp; (0.5+(grid_size(2)-2)*3/4.0)*grid_w  (idx+0.5)*grid_w 2 0];
            else
                Wp = [Wp; 0.5*grid_w  (idx+floor(idx/2)+0.5)*grid_w 2 0];
     
                Wp = [Wp; (0.5+(grid_size(2)-2)*1/4.0)*grid_w  (idx+2.5)*grid_w 2 0];
                Wp = [Wp; (0.5+(grid_size(2)-2)*2/4.0)*grid_w  (idx+2.5)*grid_w 2 0];
                Wp = [Wp; (0.5+(grid_size(2)-2)*3/4.0)*grid_w  (idx+2.5)*grid_w 2 0];
            end
        end
        if (mod(idx, 4) == 2)
            Wp = [Wp; (grid_size(2) - 1.5)*grid_w  (idx+floor(idx/2)-1.5)*grid_w 2 0];
        end
        if (mod(idx, 4) == 3)
            if (idx == 7)
                
                Wp = [Wp; (grid_size(2) - 1.5)*grid_w (idx+floor(idx/2)-0.5)*grid_w 2 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*1/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 2 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*2/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 2 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*3/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 2 0];
            else
                Wp = [Wp; (grid_size(2) - 1.5)*grid_w (idx+floor(idx/2)-0.5)*grid_w 1 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*1/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 1 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*2/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 1 0];
                Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*3/4.0)*grid_w (idx+floor(idx/2)-0.5)*grid_w 1 0];
            end
        end
        if (mod(idx, 4) == 0)
            if (idx == 8)
                Wp = [Wp; 0.5*grid_w  (idx+floor(idx/2)-2.5)*grid_w 2 0];
            else
                 Wp = [Wp; 0.5*grid_w  (idx+floor(idx/2)-2.5)*grid_w 1 0];
            end
        end
    end
 elseif (strcmp(navigation_mode,'Point03'))
    disp('Generating waypoints...')
    wp_current = 1;
    for idx = 1: grid_size(1)-2
        
            if (idx == 1)
                Wp = [Wp; 0.5*grid_w  1.5*grid_w 2 1];
                Wp = [Wp; 2.5*grid_w  1.5*grid_w 2 0];
                Wp = [Wp; 4.5*grid_w  1.5*grid_w 2 0];
                Wp = [Wp; 6.5*grid_w  1.5*grid_w 2 0];
                Wp = [Wp; 8.5*grid_w  1.5*grid_w 2 0];
            elseif (idx == 2)
                Wp = [Wp; 8.5*grid_w  3.5*grid_w 1 0];
            elseif (idx == 3)
                Wp = [Wp; 8.5*grid_w  3.5*grid_w 1 0];
                Wp = [Wp; 7.5*grid_w  3.5*grid_w 1 0];
                Wp = [Wp; 6.5*grid_w  3.5*grid_w 3 0];
                Wp = [Wp; 4.5*grid_w  3.5*grid_w 3 0];
            elseif (idx == 4)   
                Wp = [Wp; 2.5*grid_w  3.5*grid_w 3 0];  
                Wp = [Wp; 0.5*grid_w  3.5*grid_w 3 0];  
            elseif (idx == 5)
                Wp = [Wp; 0.5*grid_w  3.5*grid_w 3 0];
                Wp = [Wp; 2.5*grid_w  3.5*grid_w 3 0];
                Wp = [Wp; 4.5*grid_w  3.5*grid_w 3 0];
                Wp = [Wp; 4.5*grid_w  3.5*grid_w 2 0];
                Wp = [Wp; 4.5*grid_w  7.5*grid_w 2 0];
                Wp = [Wp; 2.5*grid_w  7.5*grid_w 2 0];
                Wp = [Wp; 0.5*grid_w  7.5*grid_w 2 0];
                Wp = [Wp; 2.5*grid_w  7.5*grid_w 2 0];
                Wp = [Wp; 4.5*grid_w  7.5*grid_w 2 0];
                Wp = [Wp; 6.5*grid_w  7.5*grid_w 2 0];
                Wp = [Wp; 8.5*grid_w  7.5*grid_w 2 0];
            end
    end
elseif (strcmp(navigation_mode,'Point04'))
    disp('Generating waypoints...')
    wp_current = 1;
    starting_grid = [5 6];
    pos_uwb_offset = [25*4+12.5 25+12.5];
                
        Wp = [Wp; 4.5*grid_w  1.5*grid_w 2 1];
        Wp = [Wp; 6.5*grid_w  1.5*grid_w 2 0];
        Wp = [Wp; 8.5*grid_w  1.5*grid_w 2 0];
        Wp = [Wp; 7.5*grid_w  1.5*grid_w 1 0];
        Wp = [Wp; 8.5*grid_w  1.5*grid_w 1 0];
        Wp = [Wp; 8.5*grid_w  3.5*grid_w 1 0];
        Wp = [Wp; 8.5*grid_w  3.5*grid_w 1 0];
        Wp = [Wp; 5.5*grid_w  3.5*grid_w 1 0];
        Wp = [Wp; 4.5*grid_w  3.5*grid_w 1 0];  
        Wp = [Wp; 4.5*grid_w  3.5*grid_w 3 0];  
        Wp = [Wp; 4.5*grid_w  4.5*grid_w 3 0];
        Wp = [Wp; 2.5*grid_w  4.5*grid_w 3 0];
        Wp = [Wp; 1.5*grid_w  4.5*grid_w 3 0];
        Wp = [Wp; 0.5*grid_w  4.5*grid_w 3 0];
        Wp = [Wp; 0.5*grid_w  2.5*grid_w 11 0];
        Wp = [Wp; 0.5*grid_w  1.5*grid_w 11 0];
        
        
    Map_obs = [0 0; 2 1; 3 1; 4 1; 4 2; 4 3];
else
    disp('Navigation method is invalid.')
    disp('Terminating Matlab script...')
    return
end

%% DRAW MAP
figure(1)
axis([-grid_w grid_w*(grid_size(2)+1) -grid_w grid_w*(10+1)])
hold on

 % Draw Waypoints
if (is_display_wp)
    for idx = 1: size(Wp,1)
        Circle_Wp(idx) = plot(Wp(idx, 1), Wp(idx, 2),'Color', 'r', 'LineWidth', 2, 'Marker', 'o');
    end
end
txt_endLine_2 = [0 0];
txt_endLine_2_last = [0 0];

txt_endLine_1 = [0 0];
txt_endLine_1_last = [0 0];



        for idxobs = 2:size(Map_obs,1)
                Obstacles = rectangle('Position', [grid_w*(Map_obs(idxobs, :) - [1 1]), grid_w grid_w], ...
                                                'FaceColor', [0 0 0]);
        end
%% Square Waypoint  (SW)
tic
if ( strcmp( Algorithm, 'square_waypoint'))
    
    % Algorithm Setup
    occup_start = [2 1; 1 1; 1 2; 2 2];
    
    for idx = 1: 4
        Grid_current( occup_start( idx, 1), occup_start( idx, 2), 1) = idx ;
        Grid_visited( occup_start( idx, 1), occup_start( idx, 2), 1) = 1;
    end 
    pos_uwb_raw(:, 1) = pos_uwb_offset;
    pos_uwb(:, 1) = pos_uwb_raw(:, 1);

    % Algorithm Main Loop
    for step = 1:max_step
        
        % Pause function
        pause(time_pause);
           
        % streaming input
        if (is_streaming_on)
           
            fid = fopen('C:\Marvelmind\dashboard\logs\TestLog.txt','rt');
            txt_Streaming = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', 'delimiter', ',','collectoutput',true...
                  , 'HeaderLines', 3);
            txt_Streaming=txt_Streaming{1};
            fclose(fid);
            txt_rows=size(txt_Streaming,1);
            if  (txt_Streaming(end, 4) == 13)
                txt_endLine_2 = txt_Streaming(end, 5:6);
            end
            if (txt_Streaming(end, 4) == 2)
                txt_endLine_1 = txt_Streaming(end, 5:6);
            end
            
            if (is_fixed_offset) 
                txt_endLine_2 = txt_endLine_2*100 + fixed_offset;
                txt_endLine_1 = txt_endLine_1*100 + fixed_offset;
            else
                txt_endLine_2 = txt_endLine_2*100 + flex_offset;
                txt_endLine_1 = txt_endLine_1*100 + flex_offset;
            end
            valid_angle = 0;
            % Initialize starting position
            if (~is_pos_initialized && (txt_endLine_2(1) ~= 0 &&  ~isnan(txt_endLine_2(1))  &&  txt_endLine_2(2) ~= 0 &&  ~isnan(txt_endLine_2(2))))
                if (txt_endLine_2(1) < streaming_max_single_val && txt_endLine_2(2) < streaming_max_single_val)
                    count_pos_initialize = count_pos_initialize + 1;
                    pos_initial = [pos_initial; txt_endLine_2];
                    if count_pos_initialize >= max_pos_initialize
                        txt_endLine_2_last = mean(pos_initial);
                        is_pos_initialized = true;
                        disp(['Position initialized at x:',  num2str(txt_endLine_2_last(1)), '; y:', num2str(txt_endLine_2_last(2))]);
                        flex_offset = (starting_grid - [0.5 0.5])* grid_w - txt_endLine_2_last;
                        %pos_uwb_offset = txt_endLine_last - starting_grid * grid_w;
                        txt_endLine_2_last = starting_grid * grid_w;
                        txt_endLine_1_last = txt_endLine_2_last - [0 0.25];
                    end
                    pos_uwb(:, step+1) = pos_uwb(:, step);
                end
            elseif  (is_pos_initialized)
                % Streaming with Dashboard data
                if (~is_streaming_collect_single_values && txt_endLine_1(1) ~= 0 &&  ~isnan(txt_endLine_1(1))  &&  txt_endLine_1(2) ~= 0 &&  ~isnan(txt_endLine_1(2)) )
                    if (norm(txt_endLine_1))< 300
                     txt_endLine_1_last = txt_endLine_1;
                     pos_uwb_1(:, step+1)  = (1 - update_rate_streaming) * pos_uwb_1(:, step)  + update_rate_streaming * txt_endLine_1_last.' ;
                    else
                        pos_uwb_1(:, step+1)= pos_uwb_1(:, step);
                    end
                else
                    pos_uwb_1(:, step+1)= pos_uwb_1(:, step);
                end
                if (~is_streaming_collect_single_values && txt_endLine_2(1) ~= 0 &&  ~isnan(txt_endLine_2(1))  &&  txt_endLine_2(2) ~= 0 &&  ~isnan(txt_endLine_2(2)) )
                    if (txt_endLine_2_last(1) ~= txt_endLine_2(1) || txt_endLine_2_last(2) ~= txt_endLine_2(2)) 
                        if norm(txt_endLine_2_last - txt_endLine_2) < streaming_max_shift_dis
                            line([txt_endLine_2_last(1) txt_endLine_2(1)], [txt_endLine_2_last(2) txt_endLine_2(2)]);
                            txt_endLine_2_last = txt_endLine_2;
                            pos_uwb(:, step+1) = (1 - update_rate_streaming) * pos_uwb(:, step) + update_rate_streaming * txt_endLine_2_last.' ;
                        else
                            pos_uwb(:, step+1) = pos_uwb(:, step);
                        end
                    else
                        pos_uwb(:, step+1) = pos_uwb(:, step);
                    end
                else
                    pos_uwb(:, step+1) = pos_uwb(:, step);
                end
            else
                pos_uwb_1(:, step+1)= pos_uwb_1(:, step);
                pos_uwb(:, step+1) = pos_uwb(:, step);
            end
        end
        
        % Grid Location
        robot_Grid = [floor(pos_uwb(1, step)/grid_w) floor(pos_uwb(2, step)/grid_w)];
       
            
        % calibrate pos here
        pos_x = pos_uwb(1,step);
        pos_nx = pos_uwb(1, step+1);
        pos_y = pos_uwb(2,step);
        pos_ny = pos_uwb(2, step+1);
       

    end
end

if (is_xbee_on)
     writedata = char('S');
     fwrite(arduino,writedata,'char');
end

disp('===================');
disp('Robot Navigation Completed!');
toc
if (is_calculate_coverage)
    disp(['Final Map Coverage: ',  num2str(count_cvg_point*100 / numel(Cvg(:, 1))), ' %']);
end