function rms = robot_motion_sequence(shape)
    hcc = floor((shape-1)/7);  % Heading command compensate value;
    
    % F R B L
    switch hcc
        case 0
            rms = [0 1; 1 0; 0 -1; -1 0];
        case 1
            rms = [1 0; 0 -1; -1 0; 0 1];
        case 2
            rms = [0 -1; -1 0; 0 1; 1 0];
        case 3
            rms = [-1 0; 0 1; 1 0; 0 -1];
    end
end