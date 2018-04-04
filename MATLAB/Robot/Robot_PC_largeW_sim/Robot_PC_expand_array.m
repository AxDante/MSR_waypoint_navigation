function exp_array= Robot_PC_expand_array(rcg,hn,gcg,CLOSED,gs, shape, Gobs)
    %Function to return an expanded array
    %This function takes a node and returns the expanded list
    %of successors,with the calculated fn values.
    %The criteria being none of the successors are on the CLOSED list.
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    
    exp_array=[];
    exp_count=1;
    c2=size(CLOSED,1);%Number of elements in CLOSED including the zeros
    
    %CLOSED
    %rms =robot_motion_sequence(shape); % Movement Patterns
    
    rmc = ['F', 'R', 'B', 'L']; % Robot movement commands
    
    for idxmp = 1:size(rmc,2) 
        disp(['Moving ', rmc(idxmp), ' from Grid (', num2str(rcg(1)), ',', num2str(rcg(2)), ')']);
        [isvalid, Rg_cmd, cost] = valid_move(rcg, shape, rmc(idxmp), Gobs, gs, 1, true);
        disp(['Moved to ', num2str(Rg_cmd(2,1)), ',', num2str(Rg_cmd(2,2))]);
        if (isvalid)
            exp_array(exp_count,1) = Rg_cmd(2,1);
            exp_array(exp_count,2) = Rg_cmd(2,2);
            exp_array(exp_count,3) = hn+distance(rcg(1),rcg(2),Rg_cmd(2,1),Rg_cmd(2,2));    %cost of travelling to node
            exp_array(exp_count,4) = distance(gcg(1),gcg(2),Rg_cmd(2,1),Rg_cmd(2,2));       %distance between node and goal
            exp_array(exp_count,5) = exp_array(exp_count,3)+exp_array(exp_count,4);         %fn
            exp_count=exp_count+1;
        end%Populate the exp_array list!!!
    end%End of j for loop
end