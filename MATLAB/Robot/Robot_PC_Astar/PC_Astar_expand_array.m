
function exp_array= PC_Astar_expand_array(rcg,hn,gcg,gs, shape, Gobs)
    %Function to return an expanded array
    %This function takes a node and returns the expanded list
    %of successors,with the calculated fn values.
    %The criteria being none of the successors are on the Closed_List list.
    
    exp_array=[];
    exp_count=1;
    
    rmc = ['F', 'R', 'B', 'L']; % Robot movement commands
    
    for idxmp = 1:size(rmc,2) 
        %disp(['Moving ', rmc(idxmp), ' from Grid (', num2str(rcg(1)), ',', num2str(rcg(2)), ')']);
        [isvalid, Rg_cmd, cost] = PC_valid_move(rcg, shape, rmc(idxmp), Gobs, gs, 1, true);
        %disp(['Moved to ', num2str(Rg_cmd(2,1)), ',', num2str(Rg_cmd(2,2))]);
        if (isvalid)
            exp_array(exp_count,1) = Rg_cmd(2,1);
            exp_array(exp_count,2) = Rg_cmd(2,2);
            
            if (rcg(2) == Rg_cmd(2,2) || rcg(2)-Rg_cmd(2,2)==1)
                discost = 0.7;
            else
                discost = distance(rcg(1),rcg(2),Rg_cmd(2,1),Rg_cmd(2,2));
            end
            exp_array(exp_count,3) = hn+discost;    %cost of travelling to node
            exp_array(exp_count,4) = distance(gcg(1),gcg(2),Rg_cmd(2,1),Rg_cmd(2,2));       %distance between node and goal
            exp_array(exp_count,5) = exp_array(exp_count,3)+exp_array(exp_count,4);         %fn
            exp_count=exp_count+1;
        end%Populate the exp_array list!!!
    end%End of j for loop
end