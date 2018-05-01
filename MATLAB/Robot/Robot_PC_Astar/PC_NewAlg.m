function Wp_s = PC_NewAlg(gs, Gobs, Wp, Row_sweep, Gvis, scg, gcg, GA)

    rowsweeptot = 0;
    for rowsweepidx = 1:size(Row_sweep,1)
        
        for rowidx = rowsweeptot +1 : rowsweeptot + Row_sweep(rowsweepidx)
            for colidx = 1:gs(2)

                %[rowidx colidx]
                
            end
            rowsweeptot = rowsweeptot +1;
        end

        
        %{
        score = 0;
        Gvis_t = Gvis;

        rmc = ['F', 'R', 'B', 'L']; % Robot movement commands

        for idxmp = 1:size(rmc,2) 
            [isvalid, Rg_cmd, cost] = PC_valid_move(rcg, shape, rmc(idxmp), Gobs, gs, 1, true);
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

        totGridSweeped = 0;
        totGridMissed = 0;
        for rowidx = rowsweeptot +1 : rowsweeptot + Row_sweep(rowsweepidx)
            for colidx = 1:gs(2)
                if Gvis_t(rowidx, colidx) == 1
                    totGridSweeped = totGridSweeped + 1;
                elseif Gvis_t(rowidx, colidx) == 0
                    totGridMissed = totGridMissed + 1;
                end
            end
        end
        score = score - totGridMissed*10;
        
        
        
        
        rowsweeptot = rowsweeptot + Row_sweep(rowsweepidx);
        %}
    end
end