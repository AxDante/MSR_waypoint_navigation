function  [Dy_v_t, heading_t] = robotMovement(char_command_t, heading_t, speed_t)
    for idx_t = 1:4
        switch char_command_t
            case 'F'
                Dy_v_t(idx_t, :) = [sin(heading_t(idx_t)) cos(heading_t(idx_t))]* speed_t;
            case 'R'
                Dy_v_t(idx_t, :) = [cos(heading_t(idx_t)) -sin(heading_t(idx_t))]* speed_t;
            case 'B'
                Dy_v_t(idx_t, :) = [-sin(heading_t(idx_t)) -cos(heading_t(idx_t))]* speed_t;
            case 'L'
                Dy_v_t(idx_t, :) = [-cos(heading_t(idx_t)) sin(heading_t(idx_t))]* speed_t;
            case 'r'
                heading_t(idx_t) = heading_t(idx_t) + 1/500 * pi;
                Dy_v_t(idx_t, :) = [0 0]* speed_t;
            case 'l'
                heading_t(idx_t) = heading_t(idx_t) - 1/500 * pi;
                Dy_v_t(idx_t, :) = [0 0]* speed_t;
        end
    end
end