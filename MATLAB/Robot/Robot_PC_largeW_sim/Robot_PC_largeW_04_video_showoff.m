%% Variable Setup
grid_size = [10 10];
grid_w = 25;
max_step = 20000;
tol_wp = 20;
tol_transform = pi/50;
Dy_angv_transform = pi/8;
tol_heading = pi/7;

cvg_sample_side = [20 20];

Algorithm = 'square_waypoint';
Serial_port = 'COM12';
baudrate = 9600;

is_heading_correction = false;
is_coverage_map = false;
is_calculate_coverage = true;
is_wp_disappear_upon_reach = true;
is_grid_on = true;
is_xbee_on = true;
is_fixed_offset = true;
is_streaming_on = true;


fixed_offset = [37.5-17.7 37.5-12.9];
starting_grid = [1 2];
robot_weight = [1.5 1.5 1.5 1.5];
robot_Form = 1;

time_interval = 50;

max_pos_initialize = 5;

%% Variable initialization


heading = zeros(4, max_step);
heading(:, 1) = [0 0 pi pi];

time_pause = time_interval/100;

pos_uwb_offset = [0 0];
pos_uwb_raw =  zeros(2, max_step);
pos_uwb = zeros(2, max_step);

pos_center = zeros(4, 2, max_step);

if (is_streaming_on)
    pos_uwb_offset = fixed_offset;
end


prev_char_command = 'S';

Grid_setup = zeros(grid_size(1),  grid_size(2));
Grid_current =  zeros(grid_size(1),  grid_size(2), max_step);
Grid_visited =  zeros(grid_size(1),  grid_size(2), max_step);

Dy_force = zeros(4, 2, max_step);
Dy_a = zeros(4, 2, max_step);
Dy_v = zeros(4, 2, max_step);

Wp = [];
wp_current = 1;
Circle_Wp = [];

is_rotating = false;

Line_Robot = [];
Line_Robot_area = [];
Line_Border = [];
loc_center = [0 0];

RobotShapes = [0 0 0 0 ;
                        0 0 pi pi;
                        0 0 0 -pi;
                        -pi 0 0 0;
                        -pi 0 0 -pi;
                        -pi/2 0 pi 0;
                        -pi/2 0 pi pi/2];
                    
char_command = '';
Cvg = [];
count_cvg_point = 0;
cvg_sample_w = grid_w*[grid_size(1)/cvg_sample_side(1) grid_size(2)/cvg_sample_side(2)];
grid_dhw = sqrt(2) / 2 * grid_w;

count_pos_initialize = 0;
is_pos_initialized = false;
pos_initial = [];

if (is_xbee_on)
    delete(instrfindall);
    arduino = serial(Serial_port,'BaudRate',baudrate);
    fopen(arduino);
end

for idxx = 1:(cvg_sample_side(2)+1)
    for idxy = 1:(cvg_sample_side(1)+1)
        Cvg = [Cvg; cvg_sample_w(1)*(idxy-1) cvg_sample_w(2)*(idxx-1) 0];
    end
end

    for idx = 1: 10
        if (idx == 1)
                Wp = [Wp; 1.5*grid_w  (idx+0.5)*grid_w 2];
                Wp = [Wp; (1.5+(grid_size(2)-4)*1/4.0)*grid_w  (idx+0.5)*grid_w 2];
                Wp = [Wp; (1.5+(grid_size(2)-4)*2/4.0)*grid_w  (idx+0.5)*grid_w 2];
        end
        if (idx == 2)
            Wp = [Wp; (grid_size(2) - 4.5)*grid_w  1.5*grid_w 2];
        end
        
        if (idx == 3)
            Wp = [Wp; (grid_size(2) - 4.5)*grid_w  3.5*grid_w 2];
        end
        
        if (idx == 4 || idx == 6)
            Wp = [Wp; (grid_size(2) - 1.5)*grid_w (idx*2-4.5)*grid_w 2];
            Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*1/4.0)*grid_w  (idx*2-4.5)*grid_w 2];
            Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*2/4.0)*grid_w  (idx*2-4.5)*grid_w 2];
            Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*3/4.0)*grid_w  (idx*2-4.5)*grid_w 2];
            Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*4/4.0)*grid_w  (idx*2-4.5)*grid_w 2];
        end
        
        if (idx == 5)
            Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*4/4.0)*grid_w  (idx*2-4.5)*grid_w 2];
            Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*3/4.0)*grid_w  (idx*2-4.5)*grid_w 2];
            Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*2/4.0)*grid_w  (idx*2-4.5)*grid_w 2];
            Wp = [Wp; (grid_size(2) - 1.5 - (grid_size(2)-2)*1/4.0)*grid_w  (idx*2-4.5)*grid_w 2];
            Wp = [Wp; (grid_size(2) - 1.5)*grid_w (idx*2-4.5)*grid_w 2];
        end
        
        if (idx == 6)
            Wp = [Wp; 1.5*grid_w 7.5*grid_w 2];
        end
        
        if (idx == 7)
            Wp = [Wp; 1.5*grid_w 9.5*grid_w 2];
            Wp = [Wp; 3.5*grid_w 9.5*grid_w 2];
            Wp = [Wp; 5.5*grid_w 9.5*grid_w 2];
            Wp = [Wp; 7.5*grid_w 9.5*grid_w 2];
        end
    end

%% DRAW MAP
figure(1)
axis([-grid_w grid_w*(grid_size(1)+1) -grid_w grid_w*(grid_size(2)+1)])
hold on

 % Draw Waypoints
for idx = 1: size(Wp,1)
    Circle_Wp(idx) = plot(Wp(idx, 1), Wp(idx, 2),'Color', 'r', 'LineWidth', 2, 'Marker', 'o');
end
txt_endLine = [0 0];
txt_endLine_last = [0 0];

%% Square Waypoint  (SW)

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
           
        % Robot Dynamics
        
        %for robidx = 1:4
        %Dy_v(:, :, step) = robotMovement(char_command, heading, 2);
            %Dy_force(robidx, :, step) = (Wp(wp_current, :) - pos_uwb(:,step).') / 250;
            %Dy_a(robidx, :, step) = Dy_force(robidx, :, step)  / robot_weight(robidx);
            %Dy_v(robidx, :, step+1) = Dy_v(robidx, :, step) + Dy_a(robidx, :, step) * time_interval;
        %end
       
        %pos_uwb(:, step)  
        %Wp(wp_current, :) 
        %norm(pos_uwb(:, step) - Wp(wp_current, :))

        
        % streaming input
        
        if (is_streaming_on)
            
            fid = fopen('C:\Marvelmind\dashboard\logs\TestLog.txt','rt');
            txt_Streaming = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', 'delimiter', ',','collectoutput',true...
                  , 'HeaderLines', 3);
            txt_Streaming=txt_Streaming{1};
            fclose(fid);
            txt_rows=size(txt_Streaming,1);
            txt_endLine = txt_Streaming(end, 5:6);
            %txt_endLine = [txt_endLine(1)*100  txt_endLine(2)*100] - pos_uwb_offset;
            if (is_fixed_offset) 
                txt_endLine;
                txt_endLine = txt_endLine*100 + fixed_offset;
            end
            if (~is_pos_initialized && (txt_endLine(1) ~= 0 &&  ~isnan(txt_endLine(1))  &&  txt_endLine(2) ~= 0 &&  ~isnan(txt_endLine(2))))
                if (txt_endLine(1) < 1000 && txt_endLine(2) < 1000)
                    count_pos_initialize = count_pos_initialize+1;
                    pos_initial = [pos_initial; txt_endLine];
                    if count_pos_initialize >= max_pos_initialize
                        txt_endLine_last = mean(pos_initial);
                        is_pos_initialized = true;
                        disp('Position initialized!');
                        pos_uwb_offset = txt_endLine_last - starting_grid * grid_w;
                        txt_endLine_last = starting_grid * grid_w;
                    end
                    pos_uwb(:, step+1) = pos_uwb(:, step);
                end
            elseif  (is_pos_initialized && txt_endLine(1) ~= 0 &&  ~isnan(txt_endLine(1))  &&  txt_endLine(2) ~= 0 &&  ~isnan(txt_endLine(2)) )
                if (txt_endLine_last(1) ~= txt_endLine(1) || txt_endLine_last(2) ~= txt_endLine(2)) 
                    %txt_endLine_last
                    %txt_endLine
                    if norm(txt_endLine_last - txt_endLine) < 50
                        line([txt_endLine_last(1) txt_endLine(1)], [txt_endLine_last(2) txt_endLine(2)]);
                        txt_endLine_last = txt_endLine;
                        pos_uwb(:, step+1) = txt_endLine_last.'*0.8 + 0.2*pos_uwb(:, step);
                    else
                        pos_uwb(:, step+1) = pos_uwb(:, step);
                    end
                else
                    pos_uwb(:, step+1) = pos_uwb(:, step);
                end
            else
                 pos_uwb(:, step+1) = pos_uwb(:, step);
            end
        end
        
        if(norm(pos_uwb(:, step).' - Wp(wp_current, 1:2)) < tol_wp )
            if (is_wp_disappear_upon_reach)
                delete(Circle_Wp(wp_current));
            end
            wp_current = wp_current + 1;
        end
        
        is_require_transform = false;
        for rbtidx = 1:4
            if abs(heading(rbtidx) - RobotShapes(Wp(wp_current, 3),rbtidx)) > tol_transform
                is_require_transform = true;
            end
        end
        
        if (robot_Form ~= Wp(wp_current, 3) && is_require_transform)
            is_transforming = true;
        else 
            robot_Form = Wp(wp_current, 3);
            is_transforming = false;
        end
        
        if (~is_pos_initialized && is_streaming_on)
            pos_uwb(:, step+1) = pos_uwb(:, step);
        elseif (is_transforming)
            heading = robotTransformation(Wp(wp_current, 3), heading, RobotShapes, tol_transform, Dy_angv_transform);
            pos_uwb(:, step+1) = pos_uwb(:, step);
        elseif (heading(2) > tol_heading && is_heading_correction ) 
            [Dy_v(:, :, step), heading] = robotMovement('l', heading, 0);
            pos_uwb(:, step+1) = pos_uwb(:, step);
        elseif (heading(2) < - tol_heading && is_heading_correction) 
            [Dy_v(:, :, step), heading]  = robotMovement('r', heading, 0);
            pos_uwb(:, step+1) = pos_uwb(:, step);
        else
            %TODO: U
             if abs(Wp(wp_current, 1) - pos_uwb(1,step)) > abs(Wp(wp_current, 2) - pos_uwb(2,step)) 
                if Wp(wp_current, 1) - pos_uwb(1, step) > 0
                    char_command = 'R';
                else
                    char_command = 'L';
                end
            else
                if Wp(wp_current, 2) - pos_uwb(2,step) > 0
                    char_command = 'F';
                else
                    char_command = 'B';
                end
             end
            
            %[Dy_v(:, :, step), heading]  = robotMovement(char_command, heading, 2);
            
            if (~is_streaming_on)
                [Dy_v(:, :, step), heading]  = robotMovement(char_command, heading, 2);
                pos_uwb(:, step+1) = Dy_v(2, :, step).' * time_interval+...
                                            0.2* (pos_uwb(:, step) + rand * 10 - 5) + 0.8* pos_uwb(:, step);
            end
            
            %pos_uwb(:, step+1) = Dy_v(2, :, step).' * time_interval+...
           %                                 0.2* (pos_uwb(:, step) + rand * 10 - 5) + 0.8* pos_uwb(:, step);
           % for rbtidx = 1:4
           %     heading(rbtidx) = 0.6 * heading(rbtidx) + 0.4 * (rand - 0.5) * 0.1 ;
           % end
           %heading(2) = 0.6 * heading(2) + 0.4 * (rand - 0.5)  ;
           %heading(1) = heading(2);
           %heading(3) = heading(2);
           %heading(4) = heading(2);
        end
        
        %pos_uwb(:, step)
        % Robot Commands
        %char_command
        
        char_command
        
        if (is_xbee_on)
            if (char_command~= char(prev_char_command))
                writedata = char(char_command);
                fwrite(arduino,writedata,'char');
                disp('sending:' + char_command);
                prev_char_command = char_command;
            end
        end
        %readData = fscanf(arduino, '%c', 1)
        
        
        % update uwb here
        %pos_uwb(1, step+1) = pos_uwb(1, step) + (1+ 0.1* step)* sin(step / 5);
        %pos_uwb(2, step+1) = pos_uwb(2, step) + (1+ 0.08* step)* cos(step / 5);
        
        % calibrate pos here
        pos_x = pos_uwb(1,step);
        pos_nx = pos_uwb(1, step+1);
        pos_y = pos_uwb(2,step);
        pos_ny = pos_uwb(2, step+1);
        
        
        % remove previous robot line plot
        if (~isempty(Line_Robot))
            delete(Line_Robot)
        end
        Line_Robot = [];
        Line_Border = [];
        
        % Draw Robot
        
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
        

        
        % plot robot BG
        if (is_coverage_map == 1)
            for robidx = 1:4
            Line_Robot(robidx,1) = line([pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 - heading(robidx)) ...
                                                     pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 - heading(robidx))], ...
                                                    [pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx)) ...
                                                     pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx))], 'Color', 'yellow', 'LineWidth', 2);
            Line_Robot(robidx,2) = line([pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 -  heading(robidx))... 
                                                     pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 -  heading(robidx))], ...
                                                    [ pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx))...
                                                      pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx))], 'Color', 'yellow', 'LineWidth', 2);
            Line_Robot(robidx,3) = line([pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                                     pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 -  heading(robidx))], ...
                                                    [ pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx)) ...
                                                      pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx))], 'Color', 'yellow', 'LineWidth', 2);
            Line_Robot(robidx,4) = line([pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                                     pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 -  heading(robidx))], ...
                                                    [ pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx))...
                                                      pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx))], 'Color', 'yellow', 'LineWidth', 2);
        
             end
        end
            
        % Draw Outer Border
        Line_Border(1) = line([0 0], [0 grid_w*grid_size(1)], 'Color', 'black', 'LineWidth', 2);
        Line_Border(2) =line([0 grid_w*grid_size(2)], [0 0], 'Color', 'black', 'LineWidth', 2);
        Line_Border(3) =line([grid_w*grid_size(2) grid_w*grid_size(1)], [0 grid_w*grid_size(1)], 'Color', 'black', 'LineWidth', 2);
        Line_Border(4) =line([grid_w*grid_size(2) 0], [grid_w*grid_size(2) grid_w*grid_size(1)], 'Color', 'black', 'LineWidth', 2);
        if (is_grid_on)
            for idxx = 1:(grid_size(1) + 1)
                line(grid_w*[(idxx-1) (idxx-1)], grid_w*[0 grid_size(2)], 'Color', 'black', 'LineWidth', 0.5);
            end
            for idxy = 1:(grid_size(2) + 1)
                line(grid_w*[0 grid_size(1)], grid_w*[(idxy-1) (idxy-1)], 'Color', 'black', 'LineWidth', 0.5);
            end
        end
        
        % Draw Robot Outline
        for robidx = 1:4
            %line([pos_center(robidx,1,step) pos_center(robidx,1,step+1)], [pos_center(robidx,2,step) pos_center(robidx,2,step+1)])  
            Line_Robot(robidx,1) = line([pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 - heading(robidx)) ...
                                                     pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 - heading(robidx))], ...
                                                    [pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx)) ...
                                                     pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx))], 'Color', 'green', 'LineWidth', 1);
            Line_Robot(robidx,2) = line([pos_center(robidx, 1, step)+grid_dhw*cos(pi/4 -  heading(robidx))... 
                                                     pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 -  heading(robidx))], ...
                                                    [ pos_center(robidx, 2, step)+grid_dhw*sin(pi/4 -  heading(robidx))...
                                                      pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx))], 'Color', 'green', 'LineWidth', 1);
            Line_Robot(robidx,3) = line([pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                                     pos_center(robidx, 1, step)+grid_dhw*-sin(pi/4 -  heading(robidx))], ...
                                                    [ pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx)) ...
                                                      pos_center(robidx, 2, step)+grid_dhw*cos(pi/4 -  heading(robidx))], 'Color', 'green', 'LineWidth', 1);
            Line_Robot(robidx,4) = line([pos_center(robidx, 1, step)+grid_dhw*-cos(pi/4 -  heading(robidx)) ...
                                                     pos_center(robidx, 1, step)+grid_dhw*sin(pi/4 -  heading(robidx))], ...
                                                    [ pos_center(robidx, 2, step)+grid_dhw*-sin(pi/4 -  heading(robidx))...
                                                      pos_center(robidx, 2, step)+grid_dhw*-cos(pi/4 -  heading(robidx))], 'Color', 'green', 'LineWidth', 1);   
        end
        
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
            count_cvg_point/ numel(Cvg(:, 1));
        end
        % Draw Robot Center
        line([pos_x pos_nx], [pos_y pos_ny])
        
    end
end
