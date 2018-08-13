% -----------------------------------------------------------
% Author: AxDante <knight16729438@gmail.com>
% Singapore University of Technology and Design
% Created: April 2018
% Modified: August 2018
% -----------------------------------------------------------

%  --- Function Inputs ---
% gs: grid_size (1x2 array)
% gw: grid_width (int)
% Gobs: grid obstacle map (gs(1)xgs(2) array)
% rcg: robot center grid (1x2 array)
% Wpp: starting Waypoint profile (nx3 array)
% Rsp: Robot sweeping profile (nx2 array)
% rf: robot form
%
% --- Function Outputs ---
% Wp: generated waypoints (nx3 array)
% Wpp: final Waypoint profile (nx3 array)
%
% --- Function Variables ---
% Rsd: robot sweeping direction
% GA: grid availability cell array
% GSC: grid shape change cell array
% scg: starting center grid

% ---------------------------


function [Wp, Wpp] = PCA_generate_waypoint(gs, gw, Gobs, rcg, Wpp, Rsp, rf)

    Rsd = [1; 0; -1; 0; 1; 0; -1; 0; 1];
                     
    Gvis = zeros(gs(1),gs(2));
    
    for obsidx = 1:size(Gobs,1)
        Gvis(Gobs(obsidx,1), Gobs(obsidx,2)) = -1;
    end
  
    for rowidx = 1:gs(1)
        for colidx = 1:gs(2)
             GA{rowidx,colidx} = PCA_get_grid_availability(rowidx,colidx, gs, Gobs);
             GSC{rowidx,colidx} = PCA_get_grid_shape_change(rowidx,colidx, gs, Gobs);
        end
    end
    
    scg = rcg;
    Wp = [];
    
    for idx = 1: 4
        gcg = [ceil(Wpp(idx,1)) ceil(Wpp(idx,2))];
        if (scg(1) - gcg(1) ~= 0)
            cols = [0 0];
            rows = [Rsp(idx,1) Rsp(idx,2)];
        elseif  (scg(2) - gcg(2) ~= 0)
            cols = [gcg(1), gcg(1)];
            rows = [0 0];
        end
        [Wp_s, Gvis_best] = PC_NewAlg(gs, rf, Wpp(idx,3),Gvis, scg, gcg, GA, GSC, rows, cols, Rsd(idx)); % segemented Wp
        Wp = [Wp; Wp_s];
        scg = gcg;
        Gvis = Gvis_best;
    end
    
    % Converstion between grid based coordinate to workspace coordinates
    Wp(:, 1:2) = (Wp(:, 1:2) - 0.5)*gw;
    Wpp = (Wpp-0.5)*gw;
end