function  [Dy_v_t, heading_t] = robotMovement(char_command_t, heading_t, linear_speed)
    for idx_t = 1:4
        switch char_command_t
            case 'F'
                Dy_v_t(idx_t, :) = [sin(heading_t(idx_t)) cos(heading_t(idx_t))]* linear_speed;
            case 'R'
                Dy_v_t(idx_t, :) = [cos(heading_t(idx_t)) -sin(heading_t(idx_t))]* linear_speed;
            case 'B'
                Dy_v_t(idx_t, :) = [-sin(heading_t(idx_t)) -cos(heading_t(idx_t))]* linear_speed;
            case 'L'
                Dy_v_t(idx_t, :) = [-cos(heading_t(idx_t)) sin(heading_t(idx_t))]* linear_speed;
            case 'f'
                Dy_v_t(idx_t, :) = [sin(heading_t(idx_t)) cos(heading_t(idx_t))]* linear_speed/5;
            case 'r'
                Dy_v_t(idx_t, :) = [cos(heading_t(idx_t)) -sin(heading_t(idx_t))]* linear_speed/5;
            case 'b'
                Dy_v_t(idx_t, :) = [-sin(heading_t(idx_t)) -cos(heading_t(idx_t))]* linear_speed/5;
            case 'l'
                Dy_v_t(idx_t, :) = [-cos(heading_t(idx_t)) sin(heading_t(idx_t))]* linear_speed/5;
            case 'O'
                heading_t(idx_t) = heading_t(idx_t) + 1/500 * pi;
                Dy_v_t(idx_t, :) = [0 0]* linear_speed;
            case 'P'
                heading_t(idx_t) = heading_t(idx_t) - 1/500 * pi;
                Dy_v_t(idx_t, :) = [0 0]* linear_speed;
            case 'S'
                Dy_v_t(idx_t, :) = [0 0];
        end
    end
end