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
% rows:
% cols:
%
% --- Function Outputs ---
% Wp_s:
% Gvis_best: 
% -------------------------


function [Wp_s, Gvis_best] = PCA_stripe_navigation(gs, srf, grf, Gvis, scg, gcg, GA, GSC, rows, cols, row_sweep_dir)
    
    ccg = scg;   % Current Center Grid
    cost = 0;
    cost_best = 1000;
    Wp_best = [];
    
    Wp = [scg(1) scg(2) srf];
    extra_width = 0;
    
    rows_init = rows;
    
    
    
    while (cost_best == 1000 && extra_width < 3) 
    
        if mod(extra_width,2) == 1
            rows = [rows(1)-ceil(extra_width/2) rows(2)];
        elseif mod(extra_width,2) == 0
            rows = [rows(1) rows(2)+extra_width/2];
        end
        
        disp(['Begin navigation from (',num2str(scg(1)), ', ', num2str(scg(2)), ') to (', num2str(gcg(1)), ', ', num2str(gcg(2)) ,').']);
        [Wp_best, Wp, cost_best, cost, Gvis_best, Gvis] = PCA_recursive_backtracking(gs, ccg, gcg, GA, GSC, cost, cost_best, Wp, Wp_best, grf,...
                                                                               [0 0], rows, cols, Gvis, Gvis, rows_init, row_sweep_dir, ceil(extra_width/2), ceil(extra_width/2));
        Wp_s = Wp_best;
        disp(['End navigation from (',num2str(scg(1)), ', ', num2str(scg(2)), ') to (', num2str(gcg(1)), ', ', num2str(gcg(2)) ,').' ...
            , ' cost = ', num2str(cost_best)]);
        disp(['best Wp:']);
        Wp_best
        
        extra_width = extra_width +1;
    end
    
end