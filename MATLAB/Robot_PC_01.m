grid_size = [10 10];
grid_w = 25;
max_step = 25;

pos_uwb_offset = [5 5];
pos_uwb_raw = [0 0];
pos_uwb = [0 0];

wp_tol = 7;

Grid_setup = zeros(grid_size(1),  grid_size(2));
Grid_current =  zeros(grid_size(1),  grid_size(2), max_step);
Grid_visited =  zeros(grid_size(1),  grid_size(2), max_step);

Algorithm = 'square_zigzag';

wp = [];
loc_center = [0 0];



%% Square Zigzag Algorithm (SZA)

robot_size = 2;
sza_back_count = 0;
sza_horizontal_dir = 1;

if ( strcmp( Algorithm, 'square_zigzag'))
    occup_start = [2 1; 1 1; 1 2; 2 2];
    for idx = 1: 4
        Grid_current( occup_start( idx, 1), occup_start( idx, 2), 1) = idx ;
        Grid_visited( occup_start( idx, 1), occup_start( idx, 2), 1) = 1;
    end 

    for (step = 1:max_step)
        [clear_f, clear_b, clear_r, clear_l]  = CheckClearance(grid_size , Grid_setup, Grid_current, 1, step);
        if (clear_r && sza_horizontal_dir)
            Grid_current(:, :, step+1) = RobotShift(Grid_current, 'r' , 1, step);
        elseif (clear_l && ~sza_horizontal_dir)
            Grid_current(:, :, step+1) = RobotShift(Grid_current, 'l' , 1, step);
        elseif (clear_b && sza_back_count < 2)
            Grid_current(:, :, step+1) = RobotShift(Grid_current, 'b' , 1, step);
            sza_back_count = sza_back_count + 1;
        end

        if (sza_back_count >= 2)
            sza_back_count = 0;
            sza_horizontal_dir = ~sza_horizontal_dir;
        end
        hold on 
        imagesc(Grid_current(:,:,step))
        pause(0.2)
        %Grid_current(:,:, step)
    end
end

%% Square Obstacle Avoidance Algorithm (SOAA)


robot_size = 2;
sza_back_count = 0;
sza_horizontal_dir = 1;

if ( strcmp( Algorithm, 'square_oa'))
    occup_start = [2 1; 1 1; 1 2; 2 2];
    for idx = 1: 4
        Grid_current( occup_start( idx, 1), occup_start( idx, 2), 1) = idx ;
        Grid_visited( occup_start( idx, 1), occup_start( idx, 2), 1) = 1;
    end 

    for (step = 1:max_step)
        [clear_f, clear_b, clear_r, clear_l]  = CheckClearance(grid_size , Grid_setup, Grid_current, 1, step);
        if (clear_r && sza_horizontal_dir)
            Grid_current(:, :, step+1) = RobotShift(Grid_current, 'r' , 1, step);
        elseif (clear_l && ~sza_horizontal_dir)
            Grid_current(:, :, step+1) = RobotShift(Grid_current, 'l' , 1, step);
        elseif (clear_b && sza_back_count < 2)
            Grid_current(:, :, step+1) = RobotShift(Grid_current, 'b' , 1, step);
            sza_back_count = sza_back_count + 1;
        end

        if (sza_back_count >= 2)
            sza_back_count = 0;
            sza_horizontal_dir = ~sza_horizontal_dir;
        end
        hold on 
        imagesc(Grid_current(:,:,step))
        pause(0.2)
        %Grid_current(:,:, step)
    end
end
