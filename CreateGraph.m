function [E, W]= CreateGraph(row_size, col_size, obs, shape)
    
    E = [];
    W = ones(1, row*col_size);
    for rowidx = 1:row
        for colidx = 1:col_size
            E = [E; (rowidx-1)*col_size+colidx,  (rowidx-1)*col_size+colidx+1];
        end
    end
    %{
    for obsidx = 1:size(obs,1)
        obsnode = (obs(1)-1)*col+obs(2);
        for Eidx = 1:size(E,1)
            if E(Eidx,2) == obs
                W(Eidx) = 0.01;
            end
        end
    end
    %}
    
    for Eidx = 1:size(E,1)
        scndNodeIdx = E(Eidx,2);
        scndNodeGrid = [mod(E(Eidx,2), col_size), ]
        if secondNode 
        ga = checkNodeValidity(rowidx,colidx, [row col_size], Gobs);
    
    end
    
    for rowidx = 1:row
        for colidx = 1:col_size
            ga = checkNodeValidity(rowidx,colidx, [row col_size], Gobs);
            if ga(shape) == 0 % If action is invalid

            end
        end
    end
    
    
end