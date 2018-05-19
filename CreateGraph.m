function [E, W]= CreateGraph(row, col, obs)
    
    E = [];
    W = ones(1, row*col);
    for rowidx = 1:row
        for colidx = 1:col-1
            E = [E; (rowidx-1)*col+colidx,  (rowidx-1)*col+colidx+1];
        end
    end
    for obsidx = 1:size(obs,1)
        obsnode = (obs(1)-1)*col+obs(2);
        for Eidx = 1:size(E,1)
            if E(Eidx,2) == obs
                W(Eidx) = 0.01;
            end
        end
    end
end