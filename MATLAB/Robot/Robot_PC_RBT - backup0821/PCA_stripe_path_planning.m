% -----------------------------------------------------------
% Author: AxDante <knight16729438@gmail.com>
% Singapore University of Technology and Design
% Created: April 2018
% Modified: August 2018
% -----------------------------------------------------------

%  --- Function Inputs ---
% gs: grid_size (1x2 array)
% srf: starting robot form (int)
% grf: goal robot form (int)
% Gvis: visited Grid (gs(1)xgs(2) array)
% scg: starting center grid (1x2 array)
% gcg: goal center grid (1x2 array)
% GA: grid availability cell array
% GSC: grid shape change cell array
% cols:
% cols:
%
% --- Function Outputs ---
% Wp_s:
% Gvis_best: 
% -------------------------

function [Wp_s, Gvis_best, attempt, grid_missed_num] = PCA_stripe_path_planning(gs, srf, grf, Gvis, scg, gcg, GA, GSC, cols, row_sweep_dir, stridx, Allow)
    
    ccg = scg;   % Current Center Grid
    cost = 0;
    cost_best = 1000;
    grid_miss_best = [];
    Wp_best = [];
    
    Wp = [scg(1) scg(2) srf];
    attempt = 0;
    
    cols_init = cols;
    
    while (cost_best == 1000 && attempt < 3) 
   
            
        if mod(attempt,2) == 1
            cols = [cols(1)-ceil(attempt/2) cols(2)];
            if (cols(1) <= 0)
                cols(1) = 0;
            end
        elseif mod(attempt,2) == 0
            cols = [cols(1) cols(2)+attempt/2];
            if (cols(2) >= gs(2))
                cols(2) = gs(2);
            end
        end
        
        gb = [1 gs(1) cols(1) cols(2)]
        
        disp(['Begin navigation from (',num2str(scg(1)), ', ', num2str(scg(2)), ') to (', num2str(gcg(1)), ', ', num2str(gcg(2)) ,').']);
        %[Wp_best, Wp, cost_best, cost, Gvis_best, Gvis] = PCA_recursive_backtracking(gs, ccg, gcg, GA, GSC, cost, cost_best, Wp, Wp_best, grf,...
        %                                                                       [0 0], cols, cols, Gvis, Gvis, cols_init, row_sweep_dir, ceil(extra_width/2), ceil(extra_width/2));
       
        [Wp_best, Wp, cost_best, cost, Gvis_best, Gvis, grid_miss_best, Grid_miss] = PCA_recursive_backtracking(gs, ccg, gcg, GA, GSC, cost, cost_best, Wp, Wp_best, grf,...
                                                                               [0 0], gb , Gvis, Gvis, cols_init, row_sweep_dir, Allow, grid_miss_best, stridx);

        Wp_s = Wp_best;
        disp(['End navigation from (',num2str(scg(1)), ', ', num2str(scg(2)), ') to (', num2str(gcg(1)), ', ', num2str(gcg(2)) ,').' ...
            , ' cost = ', num2str(cost_best)])
        disp(['best Wp:'])
        Wp_best
        disp(['grid missed: '])
        grid_miss_best
        
        grid_missed_num = size(grid_miss_best,1);
        attempt = attempt +1;
        
    end
    
end