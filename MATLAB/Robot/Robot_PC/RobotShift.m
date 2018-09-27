function Grid_return_t = RobotShift( Grid_current_t, shift_dir_t, heading_t, step)
    grid_size_t = size(Grid_current_t);
    Grid_return_t = zeros(grid_size_t(1), grid_size_t(2));
    shift_array = [0 0];
    switch shift_dir_t
        case 'r'
            shift_array = [0 1];
        case 'l'
            shift_array = [0 -1];
        case 'f'
            shift_array = [-1 0];
        case 'b'
            shift_array = [1 0];
    end
    
    for i = 1: grid_size_t(1)
        for j = 1: grid_size_t(2)
             if (Grid_current_t( i, j , step) > 0)
                Grid_return_t( i + shift_array(1), j + shift_array(2)) = Grid_current_t( i, j , step);
             end
        end
    end
end