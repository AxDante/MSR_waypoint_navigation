function [fc, bc, rc, lc] = CheckClearance( grid_size_t , Grid_setup_t, Grid_current_t, heading_t, step_t)
    Grid_occupied = Grid_current_t(:, :, step_t)  > 0;
    
    fc = 1;
    bc = 1;
    rc = 1;
    lc = 1;
    for i = 1: grid_size_t(1)
        for j = 1: grid_size_t(2)
            if (Grid_occupied( i, j ) == 1)
                not_at_side = 1;
                if ( i == 1) 
                    fc = 0; 
                    not_at_side = 0;
                end
                if ( j == 1) 
                    lc = 0;
                    not_at_side = 0;
                end
                if ( i == grid_size_t(1)) 
                    bc = 0;
                    not_at_side = 0;
                end
                if ( j == grid_size_t(2)) 
                    rc = 0;
                    not_at_side = 0;
                end
                if (not_at_side)
                    if (Grid_setup_t(i+1, j) ~= 0)
                        bc = 0;
                    end
                    if (Grid_setup_t(i-1, j) ~= 0)
                        fc = 0;
                    end
                    if (Grid_setup_t(i, j+1) ~= 0)
                        rc = 0;
                    end
                    if (Grid_setup_t(i, j-1) ~= 0)
                        lc = 0;
                    end
                end
            end
        end
    end
end

