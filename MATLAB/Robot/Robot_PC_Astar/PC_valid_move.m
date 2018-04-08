 function [isvalid, Rg_cmd, cost] = PC_valid_move(rcg, shape, command, Gobs, gs, cost, side_as_wall)

    Rgp = [0 -1; 0 1; 0 2;                         % Relative grid positions between modules
              0 -1; 1 0; 1 -1;
             -1 0; 0 1; 1 1;
              -1 0; 0 1; 0 2;
              1 0; 0 1; -1 1;
              1 -1; 1 0; 2 0;
              1 -1; 1 0; 2 -1];
  
   RRgp = rotationMatrix(Rgp, shape);       % Rotated Relative grid positions
   
   Rg = [ RRgp(1,:);                                % Robot Grid values
             0 0;
             RRgp(2,:);
             RRgp(3,:)] +rcg;
    
    
    rms = robot_motion_sequence(shape);  % Robot Motion Sequece Array
    
    switch command
        case 'F'
            Rg_cmd = Rg + rms(1,:);
        case 'R'
            Rg_cmd = Rg + rms(2,:);
        case 'B'
            Rg_cmd = Rg + rms(3,:);
        case 'L'
            Rg_cmd = Rg + rms(4,:);
    end
    
    % Check if the next command is valid
    isvalid = true;
    for idx = 1:size(Rg_cmd,1)
        if (side_as_wall)
            if (Rg_cmd(idx,1) > gs(1) || Rg_cmd(idx,1) <= 0 || ...
                Rg_cmd(idx,2) > gs(2) || Rg_cmd(idx,2) <= 0)
                isvalid = false;
            else
                if Gobs(Rg_cmd(idx,1), Rg_cmd(idx,2)) == 1
                    isvalid = false;
                end
            end
        end
    end
    % TODO: Update motion cost
    if (isvalid)
        cost = cost+1;
    end
end