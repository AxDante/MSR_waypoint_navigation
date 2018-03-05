function  new_heading = robotTransformation(new_shape, heading_t, RobotShapes_t, tol_t, Dy_angv_transform_t)
    new_heading = zeros(4,1);
    for idx_t = 1:4
        if (idx_t == 2)
            [heading_t(idx_t)  RobotShapes_t(new_shape, idx_t)]
        end
        if heading_t(idx_t) > RobotShapes_t(new_shape, idx_t) + tol_t
            new_heading (idx_t) = heading_t(idx_t) - Dy_angv_transform_t;
        elseif heading_t(idx_t) < RobotShapes_t(new_shape, idx_t) - tol_t
            new_heading (idx_t) = heading_t(idx_t) + Dy_angv_transform_t;
        else
            new_heading (idx_t) = heading_t(idx_t) ;
        end
    end
end