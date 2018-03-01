%% Variable Setup
grid_size = [10 10];
grid_w = 25;
max_step = 20000;
wp_tol = 7;


Algorithm = 'square_waypoint';
time_interval = 0.5;
time_pause = time_interval/50;
robot_heading = pi/8;
robot_weight = [1.5 1.5 1.5 1.5];


pos_uwb_offset = [200 165];
pos_uwb_raw =  zeros(2, max_step);
pos_uwb = zeros(2, max_step);

Grid_setup = zeros(grid_size(1),  grid_size(2));
Grid_current =  zeros(grid_size(1),  grid_size(2), max_step);
Grid_visited =  zeros(grid_size(1),  grid_size(2), max_step);

Dy_force = zeros(4, 2, max_step);
Dy_a = zeros(4, 2, max_step);
Dy_v = zeros(4, 2, max_step);

Wp = [100 200; 225 40; 50 170];
Robot_Wp_Circle = [];
Robot_Line = [];
loc_center = [0 0];

robot_Form = 2;

RobotConfig(:, :, 1) = [1 0; 0 0; -1 0; -2 0]; 
RobotConfig(:, :, 2) = [1 0; 0 0; 0 1; 1 1]; 
RobotConfig(:, :, 3) = [1 0; 0 0; -1 0; -1 -1]; 
RobotConfig(:, :, 4) = [0 1; 0 0; -1 0; -2 0]; 
RobotConfig(:, :, 5) = [1 0; 0 0; 1 0; 2 0]; 
RobotConfig(:, :, 6) = [1 0; 0 0; 1 0; 2 0]; 
RobotConfig(:, :, 7) = [1 0; 0 0; 1 0; 2 0]; 

grid_dhw = sqrt(2) / 2 * grid_w;



%% DRAW MAP!
figure(1)
axis([-grid_w grid_w*11 -grid_w grid_w*11])
hold on

% Draw Outer Border
line([0 0], [0 grid_w*grid_size(1)], 'Color', 'black', 'LineWidth', 2)
line([0 grid_w*grid_size(2)], [0 0], 'Color', 'black', 'LineWidth', 2)
line([grid_w*grid_size(2) grid_w*grid_size(1)], [0 grid_w*grid_size(1)], 'Color', 'black', 'LineWidth', 2)
line([grid_w*grid_size(2) 0], [grid_w*grid_size(2) grid_w*grid_size(1)], 'Color', 'black', 'LineWidth', 2)
    
 % Draw Waypoints
for idx = 1: size(Wp,1)
    Robot_Wp_Circle(idx) = plot(Wp(idx, 1), Wp(idx, 2),'Color', 'r', 'LineWidth', 2, 'Marker', 'o');
end


%% Square Zigzag Algorithm (SZA)

robot_size = 2;
sza_back_count = 0;
sza_horizontal_dir = 1;

if ( strcmp( Algorithm, 'square_zigzag'))
    
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
            for robidx = 1:4
                Dy_force(robidx, :, step) = ([125 125] - pos_uwb(:,step).') / 50;
                Dy_a(robidx, :, step) = Dy_force(robidx, :, step)  / robot_weight(robidx);
                Dy_v(robidx, :, step+1) = Dy_v(robidx, :, step) + Dy_a(robidx, :, step) * time_interval;
            end
        

        
        pos_uwb(:, step+1) = pos_uwb(:, step)  + Dy_v(2, :, step).' * time_interval ;

        % update uwb here
        %pos_uwb(1, step+1) = pos_uwb(1, step) + (1+ 0.1* step)* sin(step / 5);
        %pos_uwb(2, step+1) = pos_uwb(2, step) + (1+ 0.08* step)* cos(step / 5);
        
        % calibrate pos here
        pos_x = pos_uwb(1,step);
        pos_nx = pos_uwb(1, step+1);
        pos_y = pos_uwb(2,step);
        pos_ny = pos_uwb(2, step+1);
        
        
        
        % plot robot centre
        line([pos_x pos_nx], [pos_y pos_ny])
        
        % remove previous robot line plot
        if (~isempty(Robot_Line))
            delete(Robot_Line)
        end
        Robot_Line = [];
        
        % plot robot line
        for robidx = 1:4
            RobotConfig(:, :, 2) = [1 0; 0 0; 0 1; 1 1]; 
            pos_box_x = -RobotConfig(robidx,1,robot_Form)*grid_w*sin(robot_heading)+RobotConfig(robidx,2,robot_Form)*grid_w*cos(robot_heading);
            pos_box_y = -RobotConfig(robidx,1,robot_Form)*grid_w*cos(robot_heading)-RobotConfig(robidx,2,robot_Form)*grid_w*sin(robot_heading);

            Robot_Line(robidx,1) = line([pos_nx+pos_box_x+grid_dhw*cos(pi/4 - robot_heading) ...
                                                     pos_nx+pos_box_x+grid_dhw*sin(pi/4 - robot_heading)], ...
                                                    [pos_ny+pos_box_y+grid_dhw*sin(pi/4 - robot_heading) ...
                                                     pos_ny+pos_box_y-grid_dhw*cos(pi/4 - robot_heading)], 'Color', 'green', 'LineWidth', 1);
            Robot_Line(robidx,2) = line([pos_nx+pos_box_x+grid_dhw*cos(pi/4 - robot_heading)... 
                                                     pos_nx+pos_box_x-grid_dhw*sin(pi/4 - robot_heading)], ...
                                                    [pos_ny+pos_box_y+grid_dhw*sin(pi/4 - robot_heading)...
                                                     pos_ny+pos_box_y+grid_dhw*cos(pi/4 - robot_heading)], 'Color', 'green', 'LineWidth', 1);
            Robot_Line(robidx,3) = line([pos_nx+pos_box_x-grid_dhw*cos(pi/4 - robot_heading) ...
                                                     pos_nx+pos_box_x+grid_dhw*sin(pi/4 - robot_heading)], ...
                                                    [pos_ny+pos_box_y-grid_dhw*sin(pi/4 - robot_heading) ...
                                                     pos_ny+pos_box_y-grid_dhw*cos(pi/4 - robot_heading)], 'Color', 'green', 'LineWidth', 1);
            Robot_Line(robidx,4) = line([pos_nx+pos_box_x-grid_dhw*cos(pi/4 - robot_heading) ...
                                                     pos_nx+pos_box_x-grid_dhw*sin(pi/4 - robot_heading)], ...
                                                    [pos_ny+pos_box_y-grid_dhw*sin(pi/4 - robot_heading)...
                                                     pos_ny+pos_box_y+grid_dhw*cos(pi/4 - robot_heading)], 'Color', 'green', 'LineWidth', 1);
        end
        
    end
end

%% Square Waypoint  (SW)

wp_current = 1;

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
        for robidx = 1:4
            if abs(Wp(wp_current, 1) - pos_uwb(1,step)) > abs(Wp(wp_current, 2) - pos_uwb(2,step)) 
                if Wp(wp_current, 1) - pos_uwb(1,step) > 0
                    Dy_v(robidx, 1, step) = 1;
                else
                    Dy_v(robidx, 1, step) = -1;
                end
            else
                if Wp(wp_current, 2) - pos_uwb(2,step) > 0
                    Dy_v(robidx, 2, step) = 1;
                else
                    Dy_v(robidx, 2, step) = -1;
                end
            end
            %Dy_force(robidx, :, step) = (Wp(wp_current, :) - pos_uwb(:,step).') / 250;
            %Dy_a(robidx, :, step) = Dy_force(robidx, :, step)  / robot_weight(robidx);
            %Dy_v(robidx, :, step+1) = Dy_v(robidx, :, step) + Dy_a(robidx, :, step) * time_interval;
        end
        %pos_uwb(:, step)  
        %Wp(wp_current, :) 
        %norm(pos_uwb(:, step) - Wp(wp_current, :))
        if (norm(pos_uwb(:, step).' - Wp(wp_current, :)) < wp_tol)
         
            wp_current = wp_current + 1;
        end
        
        pos_uwb(:, step+1) = pos_uwb(:, step)  + Dy_v(2, :, step).' * time_interval ;

        % update uwb here
        %pos_uwb(1, step+1) = pos_uwb(1, step) + (1+ 0.1* step)* sin(step / 5);
        %pos_uwb(2, step+1) = pos_uwb(2, step) + (1+ 0.08* step)* cos(step / 5);
        
        % calibrate pos here
        pos_x = pos_uwb(1,step);
        pos_nx = pos_uwb(1, step+1);
        pos_y = pos_uwb(2,step);
        pos_ny = pos_uwb(2, step+1);
        
        
        
        % plot robot centre
        line([pos_x pos_nx], [pos_y pos_ny])
        
        % remove previous robot line plot
        if (~isempty(Robot_Line))
            delete(Robot_Line)
        end
        Robot_Line = [];
        
        % plot robot line
        for robidx = 1:4
            RobotConfig(:, :, 2) = [1 0; 0 0; 0 1; 1 1]; 
            pos_box_x = -RobotConfig(robidx,1,robot_Form)*grid_w*sin(robot_heading)+RobotConfig(robidx,2,robot_Form)*grid_w*cos(robot_heading);
            pos_box_y = -RobotConfig(robidx,1,robot_Form)*grid_w*cos(robot_heading)-RobotConfig(robidx,2,robot_Form)*grid_w*sin(robot_heading);

            Robot_Line(robidx,1) = line([pos_nx+pos_box_x+grid_dhw*cos(pi/4 - robot_heading) ...
                                                     pos_nx+pos_box_x+grid_dhw*sin(pi/4 - robot_heading)], ...
                                                    [pos_ny+pos_box_y+grid_dhw*sin(pi/4 - robot_heading) ...
                                                     pos_ny+pos_box_y-grid_dhw*cos(pi/4 - robot_heading)], 'Color', 'green', 'LineWidth', 1);
            Robot_Line(robidx,2) = line([pos_nx+pos_box_x+grid_dhw*cos(pi/4 - robot_heading)... 
                                                     pos_nx+pos_box_x-grid_dhw*sin(pi/4 - robot_heading)], ...
                                                    [pos_ny+pos_box_y+grid_dhw*sin(pi/4 - robot_heading)...
                                                     pos_ny+pos_box_y+grid_dhw*cos(pi/4 - robot_heading)], 'Color', 'green', 'LineWidth', 1);
            Robot_Line(robidx,3) = line([pos_nx+pos_box_x-grid_dhw*cos(pi/4 - robot_heading) ...
                                                     pos_nx+pos_box_x+grid_dhw*sin(pi/4 - robot_heading)], ...
                                                    [pos_ny+pos_box_y-grid_dhw*sin(pi/4 - robot_heading) ...
                                                     pos_ny+pos_box_y-grid_dhw*cos(pi/4 - robot_heading)], 'Color', 'green', 'LineWidth', 1);
            Robot_Line(robidx,4) = line([pos_nx+pos_box_x-grid_dhw*cos(pi/4 - robot_heading) ...
                                                     pos_nx+pos_box_x-grid_dhw*sin(pi/4 - robot_heading)], ...
                                                    [pos_ny+pos_box_y-grid_dhw*sin(pi/4 - robot_heading)...
                                                     pos_ny+pos_box_y+grid_dhw*cos(pi/4 - robot_heading)], 'Color', 'green', 'LineWidth', 1);
        end
        
    end
end
