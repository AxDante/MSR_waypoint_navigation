function exp_array=expand_array(rcg,hn,xTarget,yTarget,CLOSED,gs, shape, Gobs)
    %Function to return an expanded array
    %This function takes a node and returns the expanded list
    %of successors,with the calculated fn values.
    %The criteria being none of the successors are on the CLOSED list.
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    
    gs = [MAX_X MAX_Y]; %Grid Size
    exp_array=[];
    exp_count=1;
    c2=size(CLOSED,1);%Number of elements in CLOSED including the zeros
    
    %rms =robot_motion_sequence(shape); % Movement Patterns
    
    rmc = ['F', 'R', 'B', 'L']; % Robot movement commands
    
    for idxmp = 1:size(rmc,1) 
       
        [isvalid cost] = valid_move(rcg(1), rcg(2), shape, rmc(idxmp), Gobs, gs, 1, true);
        if (isvalid)
            exp_array(exp_count,1) = s_x;
            exp_array(exp_count,2) = s_y;
            exp_array(exp_count,3) = hn+1 ; %distance(node_x,node_y,s_x,s_y);%cost of travelling to node
            exp_array(exp_count,4) = 1; %distance(xTarget,yTarget,s_x,s_y);%distance between node and goal
            exp_array(exp_count,5) = exp_array(exp_count,3)+exp_array(exp_count,4);%fn
            exp_count=exp_count+1;
        end%Populate the exp_array list!!!
    end%End of j for loop
