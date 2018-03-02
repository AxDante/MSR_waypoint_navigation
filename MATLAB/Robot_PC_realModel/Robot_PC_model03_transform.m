%% Variable Setup
grid_size = [10 10];
grid_w = 25;
max_step = 20000;
wp_tol = 7;

Algorithm = 'square_waypoint';
is_coverage_map = true;

robot_Form = 3;

time_interval = 4;
time_pause = time_interval/50;

robot_weight = [1.5 1.5 1.5 1.5];


pos_uwb_offset = [25 25];
pos_uwb_raw =  zeros(2, max_step);
pos_uwb = zeros(2, max_step);

pos_center = zeros(4, 2, max_step);

heading = zeros(4, max_step);

heading = [0 0 0 0];

Grid_setup = zeros(grid_size(1),  grid_size(2));
Grid_current =  zeros(grid_size(1),  grid_size(2), max_step);
Grid_visited =  zeros(grid_size(1),  grid_size(2), max_step);

Dy_force = zeros(4, 2, max_step);
Dy_a = zeros(4, 2, max_step);
Dy_v = zeros(4, 2, max_step);

%Wp = [100 200; 225 40; 50 170];
Wp = [];
wp_current = 1;
Circle_Wp = [];

is_rotating = false;

Line_Robot = [];
Line_Robot_area = [];
Line_Border = [];
loc_center = [0 0];

RobotConfig(:, :, 1) = [1 0; 0 0; -1 0; -2 0]; 
RobotConfig(:, :, 2) = [1 0; 0 0; 0 1; 1 1]; 
RobotConfig(:, :, 3) = [1 0; 0 0; -1 0; -1 -1]; 
RobotConfig(:, :, 4) = [0 1; 0 0; -1 0; -2 0]; 
RobotConfig(:, :, 5) = [1 0; 0 0; 1 0; 2 0]; 
RobotConfig(:, :, 6) = [1 0; 0 0; 1 0; 2 0]; 
RobotConfig(:, :, 7) = [1 0; 0 0; 1 0; 2 0]; 


grid_dhw = sqrt(2) / 2 * grid_w;

    for idx = 1: 10
        if (mod(idx, 4) == 1)
            Wp = [Wp; 0.5*grid_w  (idx+0.5)*grid_w];
        end
        if (mod(idx, 4) == 2)
            Wp = [Wp; (grid_size(2) - 1.5)*grid_w  (idx-0.5)*grid_w];
        end
        if (mod(idx, 4) == 3)
            Wp = [Wp; (grid_size(2) - 1.5)*grid_w (idx+0.5)*grid_w];
        end
        if (mod(idx, 4) == 0)
            Wp = [Wp; 0.5*grid_w  (idx-0.5)*grid_w];
        end
    end
    

%% DRAW MAP!
figure(1)
axis([-grid_w grid_w*11 -grid_w grid_w*11])
hold on

 % Draw Waypoints
for idx = 1: size(Wp,1)
    Circle_Wp(idx) = plot(Wp(idx, 1), Wp(idx, 2),'Color', 'r', 'LineWidth', 2, 'Marker', 'o');
end


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
         
            if abs(Wp(wp_current, 1) - pos_uwb(1,step)) > abs(Wp(wp_current, 2) - pos_uwb(2,step)) 
                if Wp(wp_current, 1) - pos_uwb(1,step) > 0
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
            
        Dy_v(:, :, step) = robotMovement(char_command, heading, 2);
            %Dy_force(robidx, :, step) = (Wp(wp_current, :) - pos_uwb(:,step).') / 250;
            %Dy_a(robidx, :, step) = Dy_force(robidx, :, step)  / robot_weight(robidx);
            %Dy_v(robidx, :, step+1) = Dy_v(robidx, :, step) + Dy_a(robidx, :, step) * time_interval;
        %end
       
        %pos_uwb(:, step)  
        %Wp(wp_current, :) 
        %norm(pos_uwb(:, step) - Wp(wp_current, :))
        if (norm(pos_uwb(:, step).' - Wp(wp_current, :)) < wp_tol)
            delete(Circle_Wp(wp_current));
            wp_current = wp_current + 1;
        end

        
        if (heading(2) > 1/5 * pi) 
            [Dy_v, heading] = robotMovement('l', heading, 0);
            pos_uwb(:, step+1) = pos_uwb(:, step);
        elseif (heading(2) < - 1/5 * pi) 
            [Dy_v, heading]  = robotMovement('r', heading, 0);
            pos_uwb(:, step+1) = pos_uwb(:, step);
        else
            pos_uwb(:, step+1) = Dy_v(2, :, step).' * time_interval+...
                                            0.4* (pos_uwb(:, step) + rand * 10 - 5) + 0.6* pos_uwb(:, step);
           % for rbtidx = 1:4
           %     heading(rbtidx) = 0.6 * heading(rbtidx) + 0.4 * (rand - 0.5) * 0.1 ;
           % end
           heading(2) = 0.6 * heading(2) + 0.4 * (rand - 0.5)  ;
           heading(1) = heading(2);
           heading(3) = heading(2);
           heading(4) = heading(2);
        end
        
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
        % Draw Robot Center
        line([pos_x pos_nx], [pos_y pos_ny])
        
    end
end
