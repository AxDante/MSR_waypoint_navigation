function Wp = wp_generator_Shape_O_I(grid_size, grid_w, Grid_obstacle, robot_starting_shape, is_print_wp_gen_info)

    max_step = 1000;
    robot_shape = robot_starting_shape;
    dir = 1;
    
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
    
    %rotated_relative_grid_pos = rotationMatrix(Relative_grid_pos, robot_shape);
    %robot_Grid = [ rotated_relative_grid_pos(1,:);
    %                        0 0;
    %                        rotated_relative_grid_pos(2,:);
    %                        rotated_relative_grid_pos(3,:)] + rcg;
    Wp = [1 2 robot_shape];                   
    for idxrow = 1:floor(grid_size(2)/2)
        if(is_print_wp_gen_info)
            disp(['Checking Clearance for row ', num2str(idxrow*2-1), ' to ',num2str( idxrow*2)]);         
        end
        
        while_exit_counter = 0;
        end_of_row = false;
        while (~end_of_row && while_exit_counter < 25)
            rcg = Wp(end,1:2);
            robot_shape = Wp(end,3);
            while_exit_counter = while_exit_counter + 1;
            checkNextCase = true;
            
            if (dir == 1) 
                consRT = grid_size;
                consLB = [0 0];
            elseif (dir == -1)
                consRT = [0 grid_size(2)];
                consLB = [grid_size(1) 0];
            end
            if (robot_shape == 2)

                %          o o |
                %          o o |
                % ==  2 3 o |
                % ==  1 4 o |
                
                if (checkNextCase && rcg(1)+2*dir == consRT(1) && rcg(2)-1 > consLB(2))
                    if (Grid_obstacle(rcg(1)+2*dir, rcg(2)) == 0 &&...
                        Grid_obstacle(rcg(1)+2*dir, rcg(2)-1) == 0)
                        Wp = [Wp; rcg(1)+1*dir rcg(2) 2];
                        if(Grid_obstacle(rcg(1)+1*dir,    rcg(2)+1) == 0 && ...
                           Grid_obstacle(rcg(1)+2*dir, rcg(2)+1) == 0)
                            Wp = [Wp; rcg(1)+1*dir rcg(2)+1 2];
                            if(Grid_obstacle(rcg(1)+1*dir, rcg(2)+3) == 0 && ...
                               Grid_obstacle(rcg(1)+2*dir, rcg(2)+3) == 0)
                               Wp = [Wp; rcg(1)+1*dir rcg(2)+2 2];
                               dir = -1*dir;
                            end
                        end
                        checkNextCase = false;
                    end
                end
                
                % == o 2 3 o x 
                % == o 1 4 o o
                
                if (checkNextCase && rcg(1)+3*dir <= consRT(1) && rcg(2)-1 > consLB(2))
                    if (Grid_obstacle(rcg(1)+3*dir, rcg(2)) == 1 &&...
                        Grid_obstacle(rcg(1)+3*dir, rcg(2)-1) == 0 && ...
                        Grid_obstacle(rcg(1)-1*dir, rcg(2)) == 0 && ...
                        Grid_obstacle(rcg(1)-1*dir, rcg(2)-1) == 0)
                        Wp = [Wp; rcg(1) rcg(2) 8];
                        Wp = [Wp; rcg(1) rcg(2)-1 8];
                        checkNextCase = false;
                    end
                end
                
                if (checkNextCase)
                    Wp = [Wp; rcg(1)+1*dir rcg(2) 2];
                end
                
            elseif (robot_shape == 8 && dir == 1)
                if (rcg(2) == idxrow*2) % Upper I Shape
                    if (Grid_obstacle(rcg(1)-2, rcg(2)-1) == 1 &&...
                        Grid_obstacle(rcg(1)-1, rcg(2)-1) == 0 && ...
                        Grid_obstacle(rcg(1), rcg(2)-1) == 0 && ...
                        Grid_obstacle(rcg(1)+1, rcg(2)-1) == 0 && ...
                        Grid_obstacle(rcg(1)+2, rcg(2)-1) == 0 )
                        Wp = [Wp; rcg(1) rcg(2) 2];
                        Wp = [Wp; rcg(1)-1 rcg(2) 2];
                        Wp = [Wp; rcg(1) rcg(2) 2];
                    else
                        Wp = [Wp; rcg(1)+1 rcg(2) 8];
                    end
                elseif (rcg(2) == idxrow*2 - 1) % Lower I Shape
                    %  == X 
                    %  == 1 2 3 4
                    %        o o o o
                     if (checkNextCase && rcg(2) -1> 0 && rcg(1)+3 <= grid_size(2))
                         if (checkNextCase && Grid_obstacle(rcg(1)-1, rcg(2)+1) == 1 &&...
                            Grid_obstacle(rcg(1),    rcg(2)-1) == 0 && ...
                            Grid_obstacle(rcg(1)+1, rcg(2)-1) == 0 && ...
                            Grid_obstacle(rcg(1)+2, rcg(2)-1) == 0 && ...
                            Grid_obstacle(rcg(1)+3, rcg(2)-1) == 0)

                            Wp = [Wp; rcg(1) rcg(2) 2];
                            Wp = [Wp; rcg(1) rcg(2)+1 2];
                            Wp = [Wp; rcg(1)+1 rcg(2) 2];
                            checkNextCase = false;
                         end
                     end
                     
                     %  == X o o o o
                     %  ==    1 2 3 4
                     if (checkNextCase && rcg(1) > 2 && rcg(1)+2 <= grid_size(1))
                         if(Grid_obstacle(rcg(1)-2, rcg(2)+1) == 1 &&...
                            Grid_obstacle(rcg(1)-1,    rcg(2)+1) == 0 && ...
                            Grid_obstacle(rcg(1), rcg(2)+1) == 0 && ...
                            Grid_obstacle(rcg(1)+1, rcg(2)+1) == 0 && ...
                            Grid_obstacle(rcg(1)+2, rcg(2)+1) == 0)

                            Wp = [Wp; rcg(1) rcg(2)+1 8];
                            Wp = [Wp; rcg(1) rcg(2)+1 2];
                            Wp = [Wp; rcg(1)-1 rcg(2)+1 2];
                            Wp = [Wp; rcg(1) rcg(2)+1 2];
                            checkNextCase = false;
                         end
                     end
                     
                     % ==  o o o o |
                     % ==  1 2 3 4 |
                     
                     if (checkNextCase && rcg(2)+1 <= grid_size(2) && rcg(1)+3 == grid_size(1))
                         if(Grid_obstacle(rcg(1)-1,    rcg(2)+1) == 0 && ...
                            Grid_obstacle(rcg(1), rcg(2)+1) == 0 && ...
                            Grid_obstacle(rcg(1)+1, rcg(2)+1) == 0 && ...
                            Grid_obstacle(rcg(1)+2, rcg(2)+1) == 0)

                            Wp = [Wp; rcg(1) rcg(2)+1 8];
                            Wp = [Wp; rcg(1) rcg(2)+1 2];

                            checkNextCase = false;
                         end
                     end
                     
                     if (checkNextCase)
                         Wp = [Wp; rcg(1)+1 rcg(2) 8];
                     end
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
    Wp
    Wp(:, 1:2) = (Wp(:, 1:2) - 0.5)*grid_w;
end