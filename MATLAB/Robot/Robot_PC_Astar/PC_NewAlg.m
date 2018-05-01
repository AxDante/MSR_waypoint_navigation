function [Wp_s, Gvis_best] = PC_NewAlg(gs, Gobs, Wp, Gvis, scg, gcg, GA, rows, cols, row_sweep)
    
    ccg = scg;   % Current Center Grid
    cost = 0;
    cost_best = 1000;
    Wp_best = [];
    
    shape = 2;
    Wp = [scg(1) scg(2)];
    width = 0;
    
    while (cost_best == 1000 && width < 3) 
    
        rows = [rows(1)-width rows(2)]
        
        disp(['Begin navigation from (',num2str(scg(1)), ', ', num2str(scg(2)), ') to (', num2str(gcg(1)), ', ', num2str(gcg(2)) ,').']);
        [Wp_best, Wp, cost_best, cost, Gvis_best, Gvis] = recurse_cost(gs, ccg, gcg, GA, cost, cost_best, Wp, Wp_best, shape, [0 0], rows, cols, Gvis, Gvis);
        Wp_s = Wp_best;
        disp(['End navigation from (',num2str(scg(1)), ', ', num2str(scg(2)), ') to (', num2str(gcg(1)), ', ', num2str(gcg(2)) ,').' ...
            , ' cost = ', num2str(cost_best)]);
        disp(['best Wp:']);
        Wp_best
        
        width = width +1;
    end
    
end