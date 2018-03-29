function Wp = wp_generator_Shape_O_I(grid_size, grid_w, Grid_obstacle, robot_starting_grid, robot_starting_shape, is_print_wp_gen_info)

    max_step = 1000;
    robot_shape = robot_starting_shape;
    clearing_direction = 1;
    
    if(is_print_wp_gen_info)
        disp(['Begin generating waypoints for shape O I robot navigation']);
    end
    
    route_action_count = 0; % Calculate the actions required for the calculated route
    
    Relative_grid_pos = [0 -1; 0 1; 0 2;
                                  0 -1; 1 0; 1 -1;
                                 -1 0; 0 1; 1 1;
                                 -1 0; 0 1; 0 2;
                                  1 0; 0 1; -1 1;
                                  1 -1; 1 0; 2 0;
                                  1 -1; 1 0; 2 -1];
    
    rotated_relative_grid_pos = rotationMatrix(Relative_grid_pos, robot_shape);
    robot_Grid = [ rotated_relative_grid_pos(1,:);
                             0 0;
                            rotated_relative_grid_pos(2,:);
                            rotated_relative_grid_pos(3,:)] + robot_center_Grid;
    Wp = [1 2 robot_shape];                   
    for idxrow = 1:floor(grid_size(2)/2)
        if(is_print_wp_gen_info)
            disp(['Checking Clearance for row ', idxrow*2-1, ' to ', idxrow*2]);         
        end
        
        while_exit_counter = 0;
        end_of_row = false;
        while (~end_of_row || while_exit_counter < 500)
            if (robot_shape == 2 && clearing_direction == 1)
                if (robot_center_Grid(1)+3 > grid_size(1) || robot_center_Grid(2) -1 < 0)
                    can_shapeshift = false;  
                elseif (Grid_obstacle(robot_center_Grid(1)+3, robot_center_Grid(2)) == 1 &&...
                        Grid_obstacle(robot_center_Grid(1)+3, robot_center_Grid(2)-1) == 0 && ...
                        Grid_obstacle(robot_center_Grid(1)-1, robot_center_Grid(2)) == 0 && ...
                        Grid_obstacle(robot_center_Grid(1)-1, robot_center_Grid(2)-1) == 0)
                        Wp = [Wp, robot_center_Grid(1) robot_center_Grid(2)];
                end
            elseif (robot_shape == 8 && clearing_direction == 1)
                if (robot_center_Grid(2) == idxrow*2) % Upper I
                    
                elseif (robot_center_Grid(2) == idxrow*2 - 1)
                end
            else
                end_of_row = true;
            end
        end
        % checking first row
        
        % checking second row
        
    end
     %{
     for idx = 1: grid_size(2)
        if (mod(idx, 4) == 1)
            Wp = [Wp; 0.5*grid_w  (idx+0.5)*grid_w 2];
        end
        if (mod(idx, 4) == 2)
            Wp = [Wp; (grid_size(1) - 1.5)*grid_w  (idx-0.5)*grid_w 2];
        end
        if (mod(idx, 4) == 3)
            Wp = [Wp; (grid_size(1) - 1.5)*grid_w (idx+0.5)*grid_w 2];
        end
        if (mod(idx, 4) == 0)
            Wp = [Wp; 0.5*grid_w  (idx-0.5)*grid_w 2];
        end
    end
    %}
end